use super::*;

fn config() -> ApConfig {
    ApConfig { ssid: "esp32sim".into(), bssid: [2, 0x53, 0x49, 0x4d, 0, 1], channel: 6, psk: None }
}

fn station_frame(fc: u16, payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![0; 24];
    frame[..2].copy_from_slice(&fc.to_le_bytes());
    frame[4..10].copy_from_slice(&config().bssid);
    frame[10..16].copy_from_slice(&[2, 3, 4, 5, 6, 7]);
    frame.extend_from_slice(payload);
    frame
}

#[test]
fn auth_rejects_every_truncated_fixed_body() {
    let valid = station_frame(11 << 4, &[0, 0, 1, 0, 0, 0]);
    for len in 0..valid.len() {
        let mut ap = VirtualAp::new(config(), true);
        assert!(ap.on_station_tx(&valid[..len], 0).is_none());
        assert_eq!(ap.state, StaState::Idle, "length {len}");
        assert!(ap.queue.is_empty(), "length {len}");
    }
    let mut ap = VirtualAp::new(config(), false);
    assert!(ap.on_station_tx(&valid, 0).is_none());
    assert_eq!(ap.state, StaState::Authenticated);
    assert_eq!(ap.queue.len(), 1);
}

fn eapol_message4() -> Vec<u8> {
    let mut body = vec![0; 99];
    body[0] = 2;
    body[1] = 3;
    body[2..4].copy_from_slice(&95u16.to_be_bytes());
    body[4] = 2;
    body[5..7].copy_from_slice(&0x0300u16.to_be_bytes());
    body
}

fn send_eapol(ap: &mut VirtualAp, body: &[u8]) {
    ap.state = StaState::Associated;
    let mut payload = vec![0xaa, 0xaa, 3, 0, 0, 0, 0x88, 0x8e];
    payload.extend_from_slice(body);
    assert!(ap.on_station_tx(&station_frame(0x0108, &payload), 0).is_none());
}

#[test]
fn eapol_rejects_declared_lengths_shorter_than_key_header() {
    for len in 0..95u16 {
        let mut body = eapol_message4();
        body[2..4].copy_from_slice(&len.to_be_bytes());
        // Exercise the message-2 slices as well as the message-4 state transition.
        for (state, key_info) in [(WpaState::AwaitingMessage2, 0x0100u16), (WpaState::AwaitingMessage4, 0x0300)] {
            body[5..7].copy_from_slice(&key_info.to_be_bytes());
            let mut ap = VirtualAp::new(config(), false);
            ap.wpa.state = state;
            send_eapol(&mut ap, &body);
            assert_eq!(ap.wpa.state, state, "declared length {len}");
            assert!(ap.queue.is_empty());
        }
    }
}

#[test]
fn eapol_rejects_truncated_declared_payload_and_key_data() {
    for (declared, key_data) in [(96u16, 0u16), (95, 1), (u16::MAX, 0)] {
        let mut body = eapol_message4();
        body[2..4].copy_from_slice(&declared.to_be_bytes());
        body[97..99].copy_from_slice(&key_data.to_be_bytes());
        let mut ap = VirtualAp::new(config(), false);
        ap.wpa.state = WpaState::AwaitingMessage4;
        send_eapol(&mut ap, &body);
        assert_eq!(ap.wpa.state, WpaState::AwaitingMessage4);
        assert!(ap.queue.is_empty());
    }
}

#[test]
fn eapol_allows_bytes_after_declared_payload() {
    let mut body = eapol_message4();
    body.extend_from_slice(&[0xff; 8]);
    let mut ap = VirtualAp::new(config(), false);
    ap.wpa.state = WpaState::AwaitingMessage4;
    send_eapol(&mut ap, &body);
    assert_eq!(ap.wpa.state, WpaState::Installed);
}

#[test]
fn wpa_handshake_installs_keys_only_after_message4() {
    let mut cfg = config();
    cfg.psk = Some("esp32sim-pass".into());
    let mut ap = VirtualAp::new(cfg, false);
    assert_eq!(ap.wpa.state, WpaState::Idle);
    ap.on_station_tx(&station_frame(11 << 4, &[0, 0, 1, 0, 0, 0]), 0);
    ap.on_station_tx(&station_frame(0, &[1, 0, 0, 0]), 0);
    assert_eq!(ap.wpa.state, WpaState::AwaitingMessage2);
    let m1 = &ap.queue.last().unwrap().frame[32..];
    assert_eq!(&m1[5..7], &0x008au16.to_be_bytes());
    assert_eq!(&m1[81..97], &[0; 16]);
    ap.queue.clear();

    let mut m2 = eapol_message4();
    m2[5..7].copy_from_slice(&0x010au16.to_be_bytes());
    m2[17..49].fill(0x5a);
    send_eapol(&mut ap, &m2);
    assert_eq!(ap.wpa.state, WpaState::AwaitingMessage4);
    let m3 = &ap.queue.last().unwrap().frame[32..];
    assert_eq!(&m3[5..7], &0x13cau16.to_be_bytes());
    assert_ne!(&m3[81..97], &[0; 16]);
    let ethernet = [0u8; 14];
    assert_eq!(ap.data_from_ds(&ethernet).unwrap()[1] & 0x40, 0);

    send_eapol(&mut ap, &eapol_message4());
    assert_eq!(ap.wpa.state, WpaState::Installed);
    let protected = ap.data_from_ds(&ethernet).unwrap();
    assert_eq!(protected[1] & 0x40, 0x40);
    assert_eq!(protected.len(), 24 + 8 + 8 + 8); // MAC, CCMP, LLC/SNAP and MIC
}

#[test]
fn data_to_eth_rejects_every_truncated_header() {
    let mut payload = vec![0xaa, 0xaa, 3, 0, 0, 0, 0x08, 0];
    payload.extend_from_slice(&[1, 2, 3]);
    for fc in [0x0108, 0x0188] {
        let mut frame = station_frame(fc, &[]);
        if fc == 0x0188 { frame.extend_from_slice(&[0; 2]); }
        let header_len = frame.len() + 8;
        frame.extend_from_slice(&payload);
        for len in 0..header_len {
            assert!(data_to_eth(&frame[..len]).is_none(), "length {len}");
        }
        assert_eq!(data_to_eth(&frame).unwrap()[12..], [0x08, 0, 1, 2, 3]);
    }
}

#[test]
fn ap_configuration_keeps_defaults_and_recognizes_aliases() {
    let cfg = ApConfig::parse("").unwrap();
    assert_eq!(cfg.ssid, "esp32sim");
    assert_eq!(cfg.bssid, [2, 0x53, 0x49, 0x4d, 0, 1]);
    assert_eq!(cfg.channel, 6);
    assert!(cfg.psk.is_none());
    for channel in ["chan", "channel", "ch"] {
        for psk in ["psk", "password", "pass"] {
            let spec = format!("ssid=test,{channel}=11,{psk}=s=ecret,bssid=02:ab:CD:00:12:ff");
            let cfg = ApConfig::parse(&spec).unwrap();
            assert_eq!(cfg.ssid, "test");
            assert_eq!(cfg.channel, 11);
            assert_eq!(cfg.psk.as_deref(), Some("s=ecret"));
            assert_eq!(cfg.bssid, [2, 0xab, 0xcd, 0, 0x12, 0xff]);
        }
    }
}

#[test]
fn ap_configuration_rejects_unknown_options_and_invalid_values() {
    for spec in [
        "passwd=secret", "pass", "ssid=x,", "=x", "channel=0", "channel=15",
        "channel=256", "channel=-1", "channel=no", "bssid=02:53:49:4d:00",
        "bssid=02:53:49:4d:00:01:02", "bssid=02:xx:53:49:4d:00:01",
        "bssid=02:53:49:4d:00:GG", "bssid=2:53:49:4d:00:01", "bssid=+2:53:49:4d:00:01",
    ] {
        assert!(ApConfig::parse(spec).is_err(), "accepted {spec}");
    }
}

fn associated_ap() -> VirtualAp {
    let mut ap = VirtualAp::new(config(), false);
    ap.on_station_tx(&station_frame(11 << 4, &[0, 0, 1, 0, 0, 0]), 0);
    ap.on_station_tx(&station_frame(0, &[1, 0, 0, 0]), 0);
    assert_eq!(ap.state, StaState::Associated);
    ap.queue.clear();
    ap
}

fn lan_frame(n: u16) -> Vec<u8> {
    let mut e = vec![0xff; 6];
    e.extend_from_slice(&[2, 9, 9, 9, 9, 9]);
    e.extend_from_slice(&[0x08, 0x00]);
    e.extend_from_slice(&n.to_be_bytes());
    e.resize(60, 0);
    e
}

/// The S3's and C6's wifi_air_step: what is due, plus what the network sent, one frame on the air
/// per 400 us, the rest back on the queue. Returns the times beacons reached the station.
fn air(ap: &mut VirtualAp, until_us: u64, lan_per_step: u16, first_burst: u16) -> Vec<u64> {
    let mut beacons = Vec::new();
    let mut n = 0u16;
    let mut now = 0;
    while now < until_us {
        let mut due = ap.step(now);
        let burst = if now == 0 { first_burst } else { lan_per_step };
        let mut eth_rx = Vec::new();
        push_bounded(&mut eth_rx, (0..burst).map(|_| { n = n.wrapping_add(1); lan_frame(n) }), AIR_QUEUE_MAX);
        for e in eth_rx { if let Some(f) = ap.data_from_ds(&e) { due.push(AirFrame { at_us: now, frame: f }); } }
        if !due.is_empty() {
            due.sort_by_key(air_order);
            let first = due.remove(0);
            if is_beacon(&first.frame) { beacons.push(now); }
            ap.enqueue(due);
        }
        now += 400;
    }
    beacons
}

#[test]
fn beacons_keep_their_interval_under_1000_queued_frames_and_a_steady_flood() {
    // 1000 frames at once, then three per slot: more than the air delivers, for ten seconds (the
    // station would drop the link after 6 s without a beacon, esp_wifi.h:1141-1142).
    let mut ap = associated_ap();
    let beacons = air(&mut ap, 10_000_000, 3, 1000);
    let interval = ap.beacon_interval_us;
    assert!(beacons.len() as u64 >= 10_000_000 / interval - 1, "{} beacons in 10 s", beacons.len());
    let worst = beacons.windows(2).map(|w| w[1] - w[0]).max().unwrap();
    assert!(worst <= interval + 400, "a {worst} us gap between beacons");
    assert!(ap.queue.len() <= AIR_QUEUE_MAX);
    assert!(ap.queue_dropped > 900);
}

#[test]
fn only_a_management_exchange_holds_a_beacon_back() {
    let mut ap = associated_ap();
    for i in 0..10 { let f = ap.data_from_ds(&lan_frame(i)).unwrap(); ap.enqueue([AirFrame { at_us: 0, frame: f }]); }
    let due = ap.step(ap.next_beacon_us);
    assert!(due.iter().any(|a| is_beacon(&a.frame)), "queued data held the beacon back");
    ap.queue.clear();
    ap.on_station_tx(&station_frame(4 << 4, &[0, 0]), 0);                  // probe request: a response is queued
    let t = ap.next_beacon_us;
    ap.queue[0].at_us = t + 1_000;                                          // not due yet
    assert!(!ap.step(t).iter().any(|a| is_beacon(&a.frame)));
}

#[test]
fn the_air_queue_keeps_64_frames_losing_the_oldest_data_first() {
    let mut ap = associated_ap();
    ap.on_station_tx(&station_frame(4 << 4, &[0, 0]), 0);                  // one management frame
    for i in 0..100 { let f = ap.data_from_ds(&lan_frame(i)).unwrap(); ap.enqueue([AirFrame { at_us: 0, frame: f }]); }
    assert_eq!(ap.queue.len(), AIR_QUEUE_MAX);
    assert_eq!(ap.queue_dropped, 100 + 1 - AIR_QUEUE_MAX as u64);
    assert!(is_mgmt(&ap.queue[0].frame), "the management frame stays");
    let first_data = &ap.queue[1].frame;
    assert_eq!(&first_data[32..34], &(100 - 63u16).to_be_bytes(), "the newest 63 data frames stay");

    let mut q: Vec<Vec<u8>> = Vec::new();
    assert_eq!(push_bounded(&mut q, (0..100u16).map(lan_frame), AIR_QUEUE_MAX), 36);
    assert_eq!(q.len(), AIR_QUEUE_MAX);
    assert_eq!(q[0], lan_frame(36));
}

#[test]
fn the_handshake_goes_before_queued_data_and_is_never_dropped_for_it() {
    let mut cfg = config();
    cfg.psk = Some("esp32sim-pass".into());
    let mut ap = VirtualAp::new(cfg, false);
    ap.on_station_tx(&station_frame(11 << 4, &[0, 0, 1, 0, 0, 0]), 0);
    ap.on_station_tx(&station_frame(0, &[1, 0, 0, 0]), 0);                  // queues the assoc response and message 1
    ap.queue.retain(|a| !is_mgmt(&a.frame));                                // the auth and assoc responses went out
    let m1 = ap.queue.iter().find(|a| is_eapol(&a.frame)).unwrap().frame.clone();
    for i in 0..200 { let f = ap.data_from_ds(&lan_frame(i)).unwrap(); ap.enqueue([AirFrame { at_us: 0, frame: f }]); }
    assert!(ap.queue.iter().any(|a| a.frame == m1), "message 1 was dropped for data");
    assert_eq!(ap.queue.len(), AIR_QUEUE_MAX);
    let mut due = ap.step(1_000_000);
    due.sort_by_key(air_order);
    assert_eq!(due[0].frame, m1, "the handshake goes first");
    assert!(is_beacon(&due[1].frame), "then the beacon");
    assert!(due[2..].iter().all(|a| !is_mgmt(&a.frame) && !is_eapol(&a.frame)), "then data");
}
