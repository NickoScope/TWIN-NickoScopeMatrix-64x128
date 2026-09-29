//! esp32sim — the command line, one front end for every chip (`--chip s3|c3|c6`; the `esp32sim-c3`
//! and `esp32sim-c6` binaries are `--chip c3` / `--chip c6`). Parsing and everything a run does are chip-agnostic over
//! `Machine<S>`; the few flags a chip owns (board, WiFi, camera, PSRAM, register presets) live in
//! its setup function.
use esp_soc::observers::{BlockProfile, Breakpoints, Coverage, IrqLatency, MmioHeat, PcHist, RegTrace, Trace, Vcd, Watch};
use esp_soc::{Machine, Soc, SocBus, Stop};
use emu_core::{Bus, Core};
use std::path::PathBuf;

pub mod cooja;

fn usage(chip: &str) -> ! {
    eprintln!("usage: esp32sim [--chip s3|c3|c6] --boot rom|app --bootloader B.bin --ptable P.bin --app A.bin [--elf X.elf]... [options]");
    eprintln!("       see docs/cli.md for every flag (default chip here: {})", chip);
    std::process::exit(2)
}

fn usage_error(message: &str) -> ! { eprintln!("{message}"); std::process::exit(2) }

fn timing_cycles(value: &str, name: &str) -> Result<u32, String> {
    value.parse().map_err(|_| format!("{name}: expected a nonnegative u32 cycle count, got {value:?}"))
}

fn validate_timing(o: &Opts) -> Result<(), String> {
    if o.approximate_timing && !matches!(o.chip.as_str(), "s3" | "esp32s3") {
        return Err("--approximate-timing, --approximate-memory and --approximate-cache require --chip s3".into());
    }
    if o.memory_contention && o.approximate_memory.is_none() {
        return Err("--memory-contention requires --approximate-memory".into());
    }
    if o.approximate_memory.is_some() && o.boot.as_deref() != Some("rom") {
        return Err("--approximate-memory requires --boot rom for its MMU shadow".into());
    }
    Ok(())
}

fn cache_config() -> Result<esp32s3::approximate_cache::CacheConfig, String> {
    let mut cache = esp32s3::approximate_cache::CacheConfig::default();
    for (name, cycles) in [("ESP32SIM_CACHE_FILL", &mut cache.fill_cycles), ("ESP32SIM_CACHE_WRITEBACK", &mut cache.writeback_cycles)] {
        match std::env::var(name) {
            Ok(value) => *cycles = timing_cycles(&value, name)?,
            Err(std::env::VarError::NotPresent) => {},
            Err(_) => return Err(format!("{name}: expected a UTF-8 cycle count")),
        }
    }
    Ok(cache)
}

fn hex(s: &str, what: &str) -> u32 { u32::from_str_radix(s.trim_start_matches("0x"), 16).unwrap_or_else(|_| { eprintln!("--{}: bad hex {}", what, s); std::process::exit(2) }) }
fn pair(s: &str, dflt: usize) -> (u32, usize) { match s.split_once(',') { Some((a, n)) => (hex(a, "addr"), n.parse().unwrap_or(dflt)), None => (hex(s, "addr"), dflt) } }

use esp_soc::load::stub_spec;

fn console_mask(name: &str) -> u32 {
    esp_soc::Console::parse_mask(name).unwrap_or_else(|| {
        eprintln!("--console: unknown source {name:?}; expected usb, uart0, both, all or none");
        std::process::exit(2)
    })
}

/// What `--net` puts behind the virtual access point.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum NetMode {
    /// `nat` (or `user`): the virtual network, with TCP/UDP relayed through host sockets
    #[default]
    Nat,
    /// `none`: the virtual network alone; traffic past the gateway is refused
    None,
    /// `bridge:PATH`: the station's frames go to a real LAN through the socket_vmnet daemon at PATH
    Bridge(String),
}

/// `--net nat|user|none|bridge:PATH`. Anything else is an error, not quietly `none`.
pub fn net_mode(v: &str) -> Result<NetMode, String> {
    match v {
        "nat" | "user" => Ok(NetMode::Nat),
        "none" => Ok(NetMode::None),
        _ => match v.strip_prefix("bridge:") {
            Some(path) if !path.is_empty() => Ok(NetMode::Bridge(path.to_string())),
            Some(_) => Err("--net bridge:PATH needs the socket_vmnet socket, e.g. bridge:/var/run/socket_vmnet.bridged.en0".into()),
            None => Err(format!("--net {v}: expected nat, none or bridge:PATH")),
        },
    }
}

/// Everything the command line can say, chip-agnostic; `None` means "the chip's default".
#[derive(Default)]
pub struct Opts {
    pub approximate_timing: bool,
    pub approximate_memory: Option<u32>,
    pub memory_contention: bool,
    pub approximate_cache: bool,
    pub chip: String,
    pub rom: Option<PathBuf>, pub bootloader: Option<String>, pub ptable: Option<String>, pub app: Option<String>, pub elfs: Vec<String>,
    pub flash_image: Option<String>, pub flash_at: Vec<String>, pub boot: Option<String>, pub flash_mb: Option<usize>, pub psram_mb: Option<usize>, pub flash_id: Option<[u8; 3]>, pub flash_persist: Option<String>, pub cpi: Option<(u32, u32)>, pub serial_hex: Vec<u8>,
    pub mac: Option<[u8; 6]>, pub strap: Option<u32>, pub reset_cause: Option<u32>, pub efuse_regs: Option<String>, pub regs_init: Option<String>,
    pub board: String, pub wifi: Option<String>, pub net: NetMode, pub hostfwd: Vec<esp_soc::nat::HostFwd>, pub cam_image: Option<String>, pub cam_fps: f64,
    pub spi2_timing: bool, pub measured_te: bool,
    pub max_insns: u64, pub max_seconds: Option<f64>, pub script: Option<String>, pub serial: Option<String>,
    pub console: Option<String>, pub console_prefix: bool, pub realtime: bool, pub web_port: Option<u16>, pub web_dir: Option<String>, pub no_reboot: bool,
    /// `--serial-tcp PORT`: the USB-Serial/JTAG as an RFC 2217 port on 127.0.0.1:PORT
    pub serial_tcp: Option<u16>,
    pub wav: Option<String>, pub tft_png: Option<String>, pub gram_png: Option<String>, pub dump: bool,
    pub trace: bool, pub trace_from: u64, pub breaks: Vec<u32>, pub watch: Option<u32>, pub peeks: Vec<(u32, usize)>, pub disasms: Vec<(u32, usize)>,
    pub profile: bool, pub profile_blocks: bool, pub coverage: Option<Option<String>>, pub irq_latency: bool, pub vcd: Option<String>,
    pub regstat: Option<String>, pub regtrace: Option<String>, pub regtrace_max: u64, pub regtrace_from_pc: Option<u32>,
    pub stubs: Vec<String>, pub trace_fns: Vec<String>, pub stop_exc: u64, pub log_periph: bool, pub no_jit: bool, pub debug: Vec<String>,
    /// `--cooja`: run as a Cooja-NG external mote over NDJSON on stdin/stdout (ESP32-C6 only)
    pub cooja: bool, pub cooja_slice_us: u64, pub cooja_verbose: bool, pub cooja_rx_at_end: bool,
}

pub fn parse(args: &[String], default_chip: &str) -> Opts {
    let mut o = Opts { chip: default_chip.to_string(), board: "atech14".into(), net: NetMode::Nat, cam_fps: 10.0, max_insns: u64::MAX, dump: true, regtrace_max: u64::MAX, stop_exc: u64::MAX, cooja_slice_us: 100, ..Default::default() };
    let mut i = 1;
    while i < args.len() {
        let a = args[i].as_str();
        let mut next = || { i += 1; args.get(i).cloned().unwrap_or_else(|| usage(default_chip)) };
        match a {
            "--approximate-timing" => o.approximate_timing = true,
            "--approximate-memory" => { o.approximate_timing = true; o.approximate_memory = Some(timing_cycles(&next(), "--approximate-memory").unwrap_or_else(|e| usage_error(&e))); }
            "--memory-contention" => o.memory_contention = true,
            "--approximate-cache" => { o.approximate_timing = true; o.approximate_cache = true; },
            "--chip" => o.chip = next().to_ascii_lowercase(),
            "--rom" => o.rom = Some(PathBuf::from(next())),
            "--bootloader" => o.bootloader = Some(next()),
            "--ptable" => o.ptable = Some(next()),
            "--app" => o.app = Some(next()),
            "--elf" => o.elfs.push(next()),
            "--flash-image" => o.flash_image = Some(next()),
            "--flash-at" => o.flash_at.push(next()),
            "--boot" => o.boot = Some(next()),
            "--flash-mb" => o.flash_mb = Some(next().parse().expect("mb")),
            // The JEDEC ID the flash answers RDID with, six hex digits (manufacturer, type, capacity).
            // An octal (OPI) image needs a Macronix octal part: IDF 4.4 probes 0xC2 and a type of 0x8*.
            "--flash-id" => { let v = u32::from_str_radix(next().trim_start_matches("0x"), 16).expect("--flash-id: six hex digits"); o.flash_id = Some([(v >> 16) as u8, (v >> 8) as u8, v as u8]); }
            "--psram-mb" => o.psram_mb = Some(next().parse().expect("mb")),
            "--mac" => { let v = next(); let b: Vec<u8> = v.split(':').filter_map(|x| u8::from_str_radix(x, 16).ok()).collect(); if b.len() != 6 { eprintln!("--mac wants xx:xx:xx:xx:xx:xx"); std::process::exit(2); } let mut m = [0u8; 6]; m.copy_from_slice(&b); o.mac = Some(m); }
            "--strap" => o.strap = Some(hex(&next(), "strap")),
            "--reset-cause" => o.reset_cause = Some(hex(&next(), "reset-cause")),
            "--efuse-regs" => o.efuse_regs = Some(next()),
            "--regs-init" => o.regs_init = Some(next()),
            "--board" => o.board = next(),
            "--spi2-timing" => o.spi2_timing = true,
            "--measured-te" => o.measured_te = true,
            "--wifi" => o.wifi = Some(next()),
            "--net" => o.net = net_mode(&next()).unwrap_or_else(|e| usage_error(&e)),
            // A host port on 127.0.0.1 forwarded into the guest: tcp:8080-80, udp:4210-4210
            "--hostfwd" => o.hostfwd.push(esp_soc::nat::HostFwd::parse(&next()).unwrap_or_else(|e| usage_error(&format!("--hostfwd {e}")))),
            "--cam-image" => o.cam_image = Some(next()),
            "--cam-fps" => o.cam_fps = next().parse().expect("fps"),
            "--max-insns" => o.max_insns = next().replace('_', "").parse().expect("max-insns"),
            "--max-seconds" => o.max_seconds = Some(next().parse().expect("seconds")),
            "--script" => o.script = Some(next()),
            "--serial" => o.serial = Some(next()),
            // Raw bytes into the USB-Serial/JTAG console before the run, as hex (an Improv-Serial packet)
            "--serial-hex" => { let h = next(); let h = h.trim(); if h.len() % 2 != 0 { usage_error("--serial-hex: an even number of hex digits"); }
                                o.serial_hex.extend((0..h.len()).step_by(2).map(|i| u8::from_str_radix(&h[i..i + 2], 16).unwrap_or_else(|_| usage_error("--serial-hex: hex digits only")))); }
            // The flash chip as a file: loaded if it exists (the images are then only its first
            // contents), created from the images if not, and every program and erase written through.
            "--flash-persist" => o.flash_persist = Some(next()),
            // Uniform cycles per instruction on the JIT path (set_approximate_jit_timing): a rough
            // way to bring the emulated CPU's pace toward a chip whose memory stalls it.
            // A fraction is allowed (2.6): rounds alternate between 2 and 3 cycles an instruction.
            "--cpi" => { let v: f64 = next().parse().unwrap_or_else(|_| usage_error("--cpi: 1.0..256")); if !(1.0..=256.0).contains(&v) { usage_error("--cpi: 1.0..256"); }
                         let base = v.floor() as u32; o.cpi = Some((base, ((v - base as f64) * 256.0).round().min(255.0) as u32)); }
            "--console" => o.console = Some(next()),
            "--console-prefix" => o.console_prefix = true,
            "--realtime" => o.realtime = true,
            "--web" => o.web_port = Some(next().parse().expect("port")),
            "--web-dir" => o.web_dir = Some(next()),
            // The USB-Serial/JTAG as a serial port for pyserial's rfc2217:// (esptool --port
            // rfc2217://127.0.0.1:PORT): data, and DTR/RTS resets as the controller does them.
            "--serial-tcp" => o.serial_tcp = Some(next().parse().unwrap_or_else(|_| usage_error("--serial-tcp: a TCP port number"))),
            "--no-reboot" => o.no_reboot = true,
            "--wav" => o.wav = Some(next()),
            "--tft-png" => o.tft_png = Some(next()),
            "--gram-png" => o.gram_png = Some(next()),
            "--no-dump" => o.dump = false,
            "--trace" => o.trace = true,
            "--trace-from" => { o.trace = true; o.trace_from = next().replace('_', "").parse().expect("trace-from") }
            "--break" => o.breaks.push(hex(&next(), "break")),
            "--watch" => o.watch = Some(hex(&next(), "watch")),
            "--peek" => o.peeks.push(pair(&next(), 8)),
            "--disasm" => o.disasms.push(pair(&next(), 16)),
            "--profile" => o.profile = true,
            "--profile-blocks" => o.profile_blocks = true,
            "--coverage" => o.coverage = Some(None),
            "--coverage-file" => o.coverage = Some(Some(next())),
            "--irq-latency" => o.irq_latency = true,
            "--vcd" => o.vcd = Some(next()),
            "--regstat" => o.regstat = Some(next()),
            "--regtrace" => o.regtrace = Some(next()),
            "--regtrace-max" => o.regtrace_max = next().parse().expect("n"),
            "--regtrace-from-pc" => o.regtrace_from_pc = Some(hex(&next(), "regtrace-from-pc")),
            "--stub" => o.stubs.push(next()),
            "--trace-fn" => o.trace_fns.push(next()),
            "--stop-after-exceptions" => o.stop_exc = next().parse().expect("count"),
            "--log-periph" => o.log_periph = true,
            "--no-jit" => o.no_jit = true,
            "--debug" => o.debug.push(next()),
            "--cooja" => o.cooja = true,
            "--cooja-slice-us" => o.cooja_slice_us = next().parse().expect("slice-us"),
            "--cooja-verbose" => o.cooja_verbose = true,
            "--cooja-rx-timing" => o.cooja_rx_at_end = match next().as_str() { "start" => false, "end" => true, x => { eprintln!("--cooja-rx-timing {}: start or end", x); std::process::exit(2) } },
            "-h" | "--help" => usage(default_chip),
            _ => { eprintln!("unknown arg {}", a); usage(default_chip) }
        }
        i += 1;
    }
    o
}

/// `~/.espressif/tools/esp-rom-elfs/*/<name>` (the newest release wins).
fn find_rom(name: &str) -> Option<PathBuf> {
    let home = std::env::var("HOME").ok()?;
    let mut dirs: Vec<PathBuf> = std::fs::read_dir(format!("{}/.espressif/tools/esp-rom-elfs", home)).ok()?.flatten().map(|e| e.path()).collect();
    dirs.sort();
    dirs.into_iter().rev().map(|d| d.join(name)).find(|p| p.exists())
}

/// Bind every `--hostfwd` rule on the NAT, or stop with the reason a port could not be had.
fn attach_hostfwd(nat: &mut esp_soc::nat::Nat, o: &Opts) {
    for rule in &o.hostfwd {
        match nat.forward(*rule) {
            Ok(at) => eprintln!("[emu] hostfwd {} {} -> guest port {} (after the DHCP lease)", if rule.udp { "udp" } else { "tcp" }, at, rule.guest_port),
            Err(e) => usage_error(&format!("--hostfwd {}: cannot bind 127.0.0.1:{}: {}", rule, rule.host_port, e)),
        }
    }
}

/// `--hostfwd` needs the network behind `--wifi` with the NAT in front of it; a bridge needs `--wifi`.
fn check_net(o: &Opts) -> Result<(), String> {
    if let NetMode::Bridge(path) = &o.net {
        if o.wifi.is_none() { return Err(format!("--net bridge:{path} needs --wifi: the station reaches the LAN through the virtual access point")); }
        if !o.hostfwd.is_empty() { return Err(format!("--hostfwd does not go with --net bridge:{path}: the guest has its own address on the LAN, reach it there")); }
    }
    if o.hostfwd.is_empty() { return Ok(()); }
    if o.wifi.is_none() { return Err("--hostfwd needs --wifi: there is no network to forward into".into()); }
    if o.net != NetMode::Nat { return Err("--hostfwd needs --net nat".into()); }
    Ok(())
}
fn check_hostfwd(o: &Opts) { check_net(o).unwrap_or_else(|e| usage_error(&e)) }

/// Put the chosen backend behind the virtual network: the NAT (with its forwarded ports), a bridge
/// to the LAN, or nothing. Says what it did; a bridge that cannot connect ends the run.
fn attach_net(net: &mut esp_soc::net::VirtualNet, o: &Opts, station: [u8; 6], log: bool) {
    match &o.net {
        NetMode::Nat => {
            let mut nat = esp_soc::nat::Nat::new(log);
            eprintln!("[emu] NAT to the host network enabled (DNS via {}.{}.{}.{})", nat.resolver[0], nat.resolver[1], nat.resolver[2], nat.resolver[3]);
            attach_hostfwd(&mut nat, o);
            net.nat = Some(nat);
            eprintln!("[emu] virtual network: station {}.{}.{}.{}, gateway {}.{}.{}.{} (DHCP, ARP, ICMP, DNS, NTP)", net.sta_ip[0], net.sta_ip[1], net.sta_ip[2], net.sta_ip[3], net.gw_ip[0], net.gw_ip[1], net.gw_ip[2], net.gw_ip[3]);
        }
        NetMode::None => eprintln!("[emu] virtual network: station {}.{}.{}.{}, gateway {}.{}.{}.{} (DHCP, ARP, ICMP, DNS, NTP; nothing past the gateway)", net.sta_ip[0], net.sta_ip[1], net.sta_ip[2], net.sta_ip[3], net.gw_ip[0], net.gw_ip[1], net.gw_ip[2], net.gw_ip[3]),
        NetMode::Bridge(path) => {
            let b = esp_soc::net::vmnet::Bridge::connect(path, station, log).unwrap_or_else(|e| {
                eprintln!("--net bridge: {e}");
                eprintln!("  is the socket_vmnet daemon installed and running? (tools/twin/README.md in AnimatedPixelClock: \"В домашней сети\")");
                std::process::exit(2)
            });
            eprintln!("[emu] bridge to the LAN through {}: the station's frames go out as they are, the LAN's router serves DHCP; frames for {}, broadcast and multicast come back",
                      path, esp_soc::wifi::mac_str(&station));
            net.bridge = Some(b);
        }
    }
}

pub fn run_cli(default_chip: &str) {
    let args: Vec<String> = std::env::args().collect();
    let mut o = parse(&args, default_chip);
    validate_timing(&o).unwrap_or_else(|e| usage_error(&e));
    if o.approximate_cache { cache_config().unwrap_or_else(|e| usage_error(&e)); }
    if o.cooja { return run_cooja(&mut o); }
    match o.chip.as_str() {
        "s3" | "esp32s3" => { let m = setup_s3(&o); run(m, &o) }
        "c3" | "esp32c3" => { let m = setup_c3(&o); run(m, &o) }
        "c6" | "esp32c6" => { let m = setup_c6(&o); run(m, &o) }
        c => { eprintln!("--chip {}: s3, c3 or c6", c); std::process::exit(2) }
    }
}

/// `--cooja`: the C6 as a Cooja-NG external mote. csim's `hello` comes first — its node id names
/// the MAC unless `--mac` did — then the machine is set up and booted exactly as for a normal
/// run, and the NDJSON exchange takes the place of the run loop. The guest console never
/// reaches stdout: it goes to csim as `log` events. The usual report goes to stderr at the end.
fn run_cooja(o: &mut Opts) {
    if !matches!(o.chip.as_str(), "c6" | "esp32c6") { eprintln!("--cooja: only the ESP32-C6 speaks the Cooja-NG lock-step protocol"); std::process::exit(2); }
    if o.max_insns != u64::MAX { eprintln!("--cooja: --max-insns is unsupported; use --max-seconds"); std::process::exit(2); }
    let stdin = std::io::stdin();
    let mut input = stdin.lock();
    let hello = match cooja::read_hello(&mut input) { Ok(h) => h, Err(e) => { eprintln!("[cooja] {}", e); std::process::exit(2) } };
    if o.mac.is_none() { o.mac = Some(cooja::mac_for_node(hello.id)); }
    if o.console.is_none() { o.console = Some("uart0".into()); }
    let mut m = setup_c6(o);
    let boot = prepare(&mut m, o);
    let cfg = cooja::Config {
        slice_ns: o.cooja_slice_us.max(1).saturating_mul(1000),
        console_mask: console_mask(o.console.as_deref().unwrap_or("uart0")),
        rx_on_air: !o.cooja_rx_at_end,
        verbose: o.cooja_verbose,
        reboot: !o.no_reboot && boot == "rom",
    };
    eprintln!("[cooja] node {} ({}), mac {}, slice {} µs", hello.id, boot, o.mac.map(|m| m.iter().map(|b| format!("{:02x}", b)).collect::<Vec<_>>().join(":")).unwrap_or_default(), cfg.slice_ns / 1000);
    let t0 = std::time::Instant::now();
    let stdout = std::io::stdout();
    let mut output = stdout.lock();
    let summary = match cooja::run(&mut m, cfg, &hello, &mut input, &mut output) { Ok(s) => s, Err(e) => { eprintln!("[cooja] {}", e); std::process::exit(1) } };
    let dt = t0.elapsed().as_secs_f64();
    eprintln!("[cooja] {} steps, {} early yields, {} tx, {} rx ({} dropped), {} log lines; {:.3} s simulated in {:.1} s wall ({} cycles, {:.1} Mcycle/s)",
              summary.steps, summary.yields, summary.tx, summary.rx, summary.rx_dropped, summary.logs, summary.sim_ns as f64 / 1e9, dt, m.bus.cycles(), m.bus.cycles() as f64 / dt / 1e6);
    report(&mut m, o, match summary.stopped { Some(_) => Stop::Halted, None => Stop::MaxInsns }, dt);
}

/// The base MACs a run has without `--mac`; the WiFi station uses the base MAC.
const S3_MAC: [u8; 6] = [0x44, 0x1b, 0xf6, 0x75, 0xdc, 0xe0];
const C6_MAC: [u8; 6] = [0xdc, 0x1e, 0xd5, 0x6e, 0x8c, 0xdc];

fn setup_s3(o: &Opts) -> esp32s3::Machine {
    check_hostfwd(o);
    let mut m = esp32s3::machine(o.mac.unwrap_or(S3_MAC));
    m.bus.board = esp32s3::board::make_board(&o.board).unwrap_or_else(|| { eprintln!("unknown board '{}' (atech14, waveshare-cam, waveshare-lcd4b, waveshare-amoled18-v2, hub75-panel, none)", o.board); std::process::exit(2) });
    if o.measured_te {
        assert_eq!(m.bus.board.name(), "waveshare-amoled18-v2", "--measured-te requires the AMOLED V2 board");
        m.bus.board = Box::new(esp32s3::board::WaveshareAmoled18V2::with_measured_te());
    }
    m.bus.spi2_timing = o.spi2_timing;
    m.bus.attach_board_devices();
    if !o.debug.is_empty() { let mut f = esp_soc::DebugFlags::from_env(); for d in &o.debug { f.parse(d); } m.set_debug(&f); }
    if let Some(spec) = &o.wifi {
        let cfg = esp32s3::wifi::ApConfig::parse(spec).unwrap_or_else(|e| { eprintln!("--wifi: {e}"); std::process::exit(2) });
        eprintln!("[emu] virtual AP '{}' bssid {} channel {} ({})", cfg.ssid, esp32s3::wifi::mac_str(&cfg.bssid), cfg.channel, if cfg.psk.is_some() { "WPA2-PSK" } else { "open" });
        m.bus.periph.wifi.ap = Some(esp32s3::wifi::VirtualAp::new(cfg, m.bus.debug.has("wifi-frames")));
        let mut net = esp32s3::net::VirtualNet::new(m.bus.debug.has("net"));
        attach_net(&mut net, o, o.mac.unwrap_or(S3_MAC), m.bus.debug.has("net"));
        m.bus.periph.wifi.net = Some(net);
        m.bus.refresh_tick_budget();
    }
    if let Some(p) = &o.cam_image { match esp_soc::picture::load(p) { Ok(pic) => { eprintln!("[emu] camera picture {} ({}x{})", p, pic.w, pic.h); m.bus.board.set_camera_picture(pic); } Err(e) => { eprintln!("[emu] {}", e); std::process::exit(2); } } }
    m.bus.periph.lcd_cam.frame_cycles = (esp32s3::periph::CPU_HZ as f64 / o.cam_fps) as u64;
    if let Some(mb) = o.flash_mb { if mb != 8 { m.bus.set_flash_size(mb << 20); } }
    if let Some(id) = o.flash_id { m.bus.periph.spi0.jedec = id; m.bus.periph.spi1.jedec = id; }
    if let Some(mb) = o.psram_mb {
        if mb != 2 { m.bus.set_psram_size(mb << 20).unwrap(); }
        if esp_periph::opi_psram_density(mb << 20).is_none() { eprintln!("[emu] --psram-mb {}: the octal PSRAM's MR2 can only report 4, 8, 16 or 32 MB; IDF will size it 8 MB, the reset value's 64 Mbit", mb); }
    }
    if let Some(p) = &o.efuse_regs {
        let txt = std::fs::read_to_string(p).expect("efuse file");
        let mut n = 0;
        for line in txt.lines() {
            let line = line.trim(); if line.is_empty() { continue; }
            let (addr_s, rest) = match line.split_once(':') { Some(x) => x, None => continue };
            let Ok(mut a) = u32::from_str_radix(addr_s.trim().trim_start_matches("0x"), 16) else { continue };
            for w in rest.split_whitespace() { if let Ok(v) = u32::from_str_radix(w, 16) { let off = if a >= 0x6000_7000 { a - 0x6000_7000 } else { a }; m.bus.periph.efuse.ram.write(off, v); a += 4; n += 1; } }
        }
        eprintln!("[emu] loaded {} efuse words from {}", n, p);
    }
    if let Some(p) = &o.regs_init {
        let txt = std::fs::read_to_string(p).expect("regs-init file");
        let mut n = 0;
        for line in txt.lines() {
            let (addr_s, rest) = match line.trim().split_once(':') { Some(x) => x, None => continue };
            let Ok(mut a) = u32::from_str_radix(addr_s.trim().trim_start_matches("0x"), 16) else { continue };
            for w in rest.split_whitespace() { if let Ok(v) = u32::from_str_radix(w, 16) { if m.bus.periph.init_regs(a, v) { n += 1; } a += 4; } }
        }
        eprintln!("[emu] applied {} reset-state register words from {}", n, p);
    }
    if let Some(p) = &o.regstat { m.add_observer(Box::new(MmioHeat::new(p, |a| { let b = a.wrapping_sub(esp32s3::periph::PERIPH_BASE) >> 12; format!("{}+0x{:03x}", esp32s3::periph::Peripherals::block_name_pub(b), a & 0xfff) }))); }
    m
}

fn setup_c3(o: &Opts) -> esp32c3::Machine {
    let mut m = esp32c3::machine(o.mac.unwrap_or([0x60, 0x55, 0xf9, 0x00, 0x11, 0x22]), o.flash_mb.unwrap_or(4) << 20);
    m.bus.set_flash_size(o.flash_mb.unwrap_or(4) << 20);   // the JEDEC capacity follows the size
    if !o.debug.is_empty() { let mut f = esp_soc::DebugFlags::from_env(); for d in &o.debug { f.parse(d); } m.set_debug(&f); }
    for (flag, on) in [("--board", o.board != "atech14" && o.board != "none"), ("--wifi", o.wifi.is_some()), ("--hostfwd", !o.hostfwd.is_empty()), ("--cam-image", o.cam_image.is_some()), ("--psram-mb", o.psram_mb.is_some()), ("--efuse-regs", o.efuse_regs.is_some()), ("--regs-init", o.regs_init.is_some()), ("--regstat", o.regstat.is_some())] {
        if on { eprintln!("{} is not available on the C3", flag); std::process::exit(2); }
    }
    m
}

fn setup_c6(o: &Opts) -> esp32c6::Machine {
    check_hostfwd(o);
    let mut m = esp32c6::machine(o.mac.unwrap_or(C6_MAC), o.flash_mb.unwrap_or(4) << 20);
    m.bus.set_flash_size(o.flash_mb.unwrap_or(4) << 20);   // the JEDEC capacity follows the size
    if !o.debug.is_empty() { let mut f = esp_soc::DebugFlags::from_env(); for d in &o.debug { f.parse(d); } m.set_debug(&f); }
    let name = if o.board == "atech14" { "none" } else { o.board.as_str() };   // the S3 default means "bare module" here
    match esp32c6::board::make_board(name) { Some(b) => m.bus.board = b, None => { eprintln!("--board {}: none or waveshare-c6-lcd147 on the C6", name); std::process::exit(2) } }
    if let Some(spec) = &o.wifi {
        let cfg = esp_soc::wifi::ApConfig::parse(spec).unwrap_or_else(|e| { eprintln!("--wifi: {e}"); std::process::exit(2) });
        eprintln!("[emu] virtual AP '{}' bssid {} channel {} ({})", cfg.ssid, esp_soc::wifi::mac_str(&cfg.bssid), cfg.channel, if cfg.psk.is_some() { "WPA2-PSK" } else { "open" });
        m.bus.periph.wifi_mac.ap = Some(esp_soc::wifi::VirtualAp::new(cfg, m.bus.debug.has("wifi-frames")));
        let mut net = esp_soc::net::VirtualNet::new(m.bus.debug.has("net"));
        attach_net(&mut net, o, o.mac.unwrap_or(C6_MAC), m.bus.debug.has("net"));
        m.bus.periph.wifi_mac.net = Some(net);
    }
    for (flag, on) in [("--cam-image", o.cam_image.is_some()), ("--psram-mb", o.psram_mb.is_some()), ("--efuse-regs", o.efuse_regs.is_some()), ("--regs-init", o.regs_init.is_some()), ("--regstat", o.regstat.is_some())] {
        if on { eprintln!("{} is not available on the C6", flag); std::process::exit(2); }
    }
    m
}

/// Everything after the chip is set up: images, boot, observers, the run, the reports.
fn run<S: Soc>(mut m: Machine<S>, o: &Opts) {
    let approximate = o.approximate_timing.then(|| {
        let mut config = esp32s3::ApproximateTimingConfig::default();
        if o.approximate_cache && o.approximate_memory.is_none() {
            config.data_cache = Some(cache_config().unwrap_or_else(|e| usage_error(&e)));
        }
        esp32s3::ApproximateCostModel::new(config)
    });
    let memory_model = o.approximate_memory.map(|extra| {
        use esp32s3::rough_memory::{MemoryConfig, MemoryPrice};
        let external = MemoryPrice { latency: extra, ..MemoryPrice::FREE };
        let model = esp32s3::memory_cost_model::MemoryCostModel::new(approximate.as_ref().unwrap().clone(),
            MemoryConfig { flash: external, psram: external, contention: o.memory_contention, ..Default::default() });
        if o.approximate_cache { model.with_cache(cache_config().unwrap_or_else(|e| usage_error(&e))) } else { model }
    });
    if let Some(model) = &approximate {
        let cost: Box<dyn emu_core::CostModel> = match &memory_model {
            Some(memory) => Box::new(memory.clone()), None => Box::new(model.clone()),
        };
        m.set_cost_model(cost).unwrap_or_else(|e| usage_error(&format!("--approximate-timing: {e}")));
        eprintln!("[emu] APPROXIMATE timing: {:?}; use --boot rom; accuracy unvalidated", model.config);
    }
    if let Some((cpi, frac)) = o.cpi {
        m.set_approximate_jit_timing(cpi, 256).unwrap_or_else(|e| usage_error(&format!("--cpi: {e}")));
        m.set_approximate_cpi_fraction(frac).unwrap_or_else(|e| usage_error(&format!("--cpi: {e}")));
        eprintln!("[emu] {:.3} cycles per instruction (uniform, JIT)", cpi as f64 + frac as f64 / 256.0);
    }
    let boot = prepare(&mut m, o);
    let t0 = std::time::Instant::now();
    m.web_restart = !o.no_reboot;
    // The host's RTS=1/DTR=0 resets the chip only where the run comes back up through the ROM.
    m.usj_reset = !o.no_reboot && boot == "rom";
    if m.usj.is_some() && !m.usj_reset { eprintln!("[emu] USB-Serial/JTAG line resets are ignored in this run (--boot app or --no-reboot): data only"); }
    let stop = run_with_reboots(&mut m, o.max_insns, !o.no_reboot && boot == "rom", boot == "app");
    let dt = t0.elapsed().as_secs_f64();
    report(&mut m, o, stop, dt);
    if let Some(model) = approximate { eprintln!("[emu] approximate timing totals: {:?}", model.stats()); }
    if let Some(model) = memory_model {
        eprintln!("[emu] approximate memory totals [internal, ROM, flash, PSRAM, MMIO]: {:?}", model.memory.borrow().stats);
        if let Some(cache) = &model.cache { eprintln!("[emu] approximate physical data cache: {:?}", cache.borrow().stats()); }
    }
}

/// Run, coming back up after a chip reset where that is possible: through the ROM when it booted
/// the run (`reboot`), and for an app-mode run (`app`) when the reset was the page's Restart — the
/// chip is reset and the app mapped and entered again, as at startup. A reset the firmware asked
/// for still ends an app-mode run: there is no ROM to take it through.
fn run_with_reboots<S: Soc>(m: &mut Machine<S>, mut remaining: u64, reboot: bool, app: bool) -> Stop {
    loop {
        let before = m.run_steps();
        let stop = m.run(remaining);
        remaining = remaining.saturating_sub(m.run_steps().saturating_sub(before));
        if let Stop::SwReset = stop {
            let cause = m.bus.reset_cause();
            eprintln!("[emu] chip reset at t={:.3}s: cause {:#x} ({})", m.seconds(), cause, esp_periph::reset_cause_name(cause));
            let button = m.take_button_reset();
            if !reboot && !(button && app) { return stop; }
            if remaining == 0 { return Stop::MaxInsns; }
            m.reboot();
            if !reboot {
                match m.boot_app(0x10000) {
                    Ok(entry) => eprintln!("[emu] restart: app entry {:#010x} {}", entry, m.sym(entry)),
                    Err(e) => { eprintln!("[emu] restart: {}", e); return stop; }
                }
            }
        } else { return stop; }
    }
}

/// Images, boot, observers, scripts: everything before the first instruction. Returns the boot mode.
fn prepare<S: Soc>(m: &mut Machine<S>, o: &Opts) -> String {
    let c3 = S::CORES == 1;
    let boot = o.boot.clone().unwrap_or_else(|| if c3 { "rom".into() } else { "app".into() });
    let console = o.console.clone().unwrap_or_else(|| if c3 { "uart0".into() } else { "both".into() });
    m.bus.misc().log_unknown = o.log_periph;
    if !o.breaks.is_empty() { m.add_observer(Box::new(Breakpoints { pcs: o.breaks.clone() })); }
    if o.trace { m.add_observer(Box::new(Trace { from: o.trace_from })); }
    let rom = o.rom.clone().or_else(|| find_rom(S::ROM_ELF));
    match &rom {
        Some(r) => match std::fs::read(r) { Ok(d) => { m.load_rom(&d).expect("rom"); eprintln!("[emu] ROM loaded from {}", r.display()); } Err(e) => eprintln!("[emu] no ROM ({}): {}", r.display(), e) },
        None if boot == "rom" => { eprintln!("[emu] no {} mask ROM ELF found (pass --rom, or use --boot app)", S::NAME); std::process::exit(2) }
        None => {}
    }
    if let Some(p) = &o.flash_image { m.write_flash(0, &std::fs::read(p).expect("flash image")).unwrap(); }
    if let Some(p) = &o.bootloader { m.write_flash(0x0, &std::fs::read(p).expect("bootloader")).unwrap(); }
    if let Some(p) = &o.ptable { m.write_flash(0x8000, &std::fs::read(p).expect("ptable")).unwrap(); }
    if let Some(p) = &o.app { m.write_flash(0x10000, &std::fs::read(p).expect("app")).unwrap(); }
    for spec in &o.flash_at {
        let (off, path) = spec.split_once('=').unwrap_or_else(|| { eprintln!("--flash-at needs OFFSET=FILE"); std::process::exit(2) });
        let off = usize::from_str_radix(off.trim_start_matches("0x"), 16).unwrap_or_else(|_| { eprintln!("--flash-at: bad offset {}", off); std::process::exit(2) });
        let data = std::fs::read(path).unwrap_or_else(|e| { eprintln!("--flash-at: {}: {}", path, e); std::process::exit(2) });
        m.write_flash(off, &data).unwrap_or_else(|e| { eprintln!("--flash-at: {}", e); std::process::exit(2) });
        eprintln!("[emu] flash {:#x}: {} ({} bytes)", off, path, data.len());
    }
    if let Some(p) = &o.flash_persist {
        let size = m.bus.flash_contents().map(|f| f.len()).unwrap_or_else(|| usage_error("--flash-persist: this chip keeps no flash here"));
        match std::fs::read(p) {
            Ok(data) if data.len() == size => { m.write_flash(0, &data).unwrap(); eprintln!("[emu] flash chip {} ({} MB): its own contents, the images are ignored", p, size >> 20); }
            Ok(data) => usage_error(&format!("--flash-persist {}: {} bytes, but the flash is {}", p, data.len(), size)),
            Err(_) => { std::fs::write(p, m.bus.flash_contents().unwrap()).unwrap_or_else(|e| usage_error(&format!("--flash-persist {}: {}", p, e))); eprintln!("[emu] flash chip {} created from the images", p); }
        }
        let f = std::fs::OpenOptions::new().write(true).open(p).unwrap_or_else(|e| usage_error(&format!("--flash-persist {}: {}", p, e)));
        m.bus.set_flash_file(f).unwrap_or_else(|e| usage_error(&e));
    }
    for p in &o.elfs { m.add_symbols(&std::fs::read(p).expect("elf")).expect("elf symbols"); }
    if let Some(s) = &o.serial { m.bus.serial_input(s.as_bytes()); }
    if !o.serial_hex.is_empty() { m.bus.serial_input(&o.serial_hex); }
    for pre in &o.trace_fns {
        let n = m.trace_fns(pre);
        eprintln!("[emu] --trace-fn {}: {} functions", pre, n);
    }
    for st in &o.stubs {
        let (name, val) = stub_spec(st).unwrap_or_else(|e| { eprintln!("--stub: {e}"); std::process::exit(2) });
        let addr = m.resolve_stub(name).unwrap_or_else(|| { eprintln!("--stub: unknown symbol {}", name); std::process::exit(2) });
        eprintln!("[emu] stub {} @ {:#010x} -> returns {:#x}", name, addr, val);
        m.stubs.insert(addr, val);
    }
    if o.no_jit { for c in &mut m.cores { c.set_jit(false); } }
    match boot.as_str() {
        "app" => match m.boot_app(0x10000) { Ok(entry) => eprintln!("[emu] app boot: entry {:#010x} {}", entry, m.sym(entry)), Err(e) => { eprintln!("[emu] {}", e); std::process::exit(2) } },
        "rom" => { m.boot_rom(); eprintln!("[emu] ROM boot from reset vector {:#010x}", m.cores[0].pc()); }
        _ => { eprintln!("--boot app|rom"); std::process::exit(2); }
    }
    // Match a real board's boot conditions: the ROM prints the reset cause and the strapping-derived boot mode
    if let Some(c) = o.reset_cause { m.bus.set_reset_cause(c); }
    if let Some(v) = o.strap { m.bus.set_strap(v); }
    for &(a, n) in &o.peeks { eprintln!("[peek before run]\n{}", m.peek(a, n)); }
    m.dbg.stop_after_exceptions = o.stop_exc;
    m.console.mask = console_mask(&console);
    m.console.prefix = o.console_prefix;
    if let Some(p) = &o.regtrace { m.add_observer(Box::new(RegTrace::new(std::fs::File::create(p).expect("regtrace file"), o.regtrace_max, o.regtrace_from_pc))); }
    if let Some(port) = o.web_port {
        let dir = o.web_dir.clone().unwrap_or_else(|| { let exe = std::env::current_exe().unwrap(); let mut d = exe.parent().unwrap().to_path_buf(); for _ in 0..3 { if d.join("web").exists() { break; } d = d.parent().unwrap().to_path_buf(); } d.join("web").to_string_lossy().to_string() });
        let w = esp_soc::web::WebServer::start(port, dir.clone()).expect("web server");
        eprintln!("[emu] board UI: http://127.0.0.1:{}/  (serving {})", port, dir);
        m.web = Some(w); m.rt.enabled = true;
    }
    // One USB-Serial/JTAG host end for both front ends: one client at a time holds it.
    if o.web_port.is_some() || o.serial_tcp.is_some() {
        let port = esp_soc::usj_port::UsjPort::new();
        if let Some(w) = &m.web { w.attach_usj(port.clone()); eprintln!("[emu] USB-Serial/JTAG port: ws://127.0.0.1:{}/usj", w.port); }
        if let Some(tcp) = o.serial_tcp {
            match esp_soc::rfc2217::start(tcp, port.clone()) {
                Ok(p) => eprintln!("[emu] USB-Serial/JTAG port: rfc2217://127.0.0.1:{p}"),
                Err(e) => usage_error(&format!("--serial-tcp {tcp}: cannot listen on 127.0.0.1:{tcp}: {e}")),
            }
        }
        m.attach_usj(port);
    }
    if o.realtime { m.rt.enabled = true; }
    if o.profile { m.add_observer(Box::new(PcHist::new(12))); }
    if let Some(wa) = o.watch { let v = m.bus.read32_unpriced(wa).unwrap_or(0); m.add_observer(Box::new(Watch { addr: wa, value: v })); }
    if o.profile_blocks { m.add_observer(Box::new(BlockProfile::new(20))); }
    if let Some(path) = &o.coverage { m.add_observer(Box::new(Coverage::new(path.clone()))); }
    if o.irq_latency { m.add_observer(Box::new(IrqLatency::new(S::CORES))); }
    if let Some(p) = &o.vcd { m.add_observer(Box::new(Vcd::new(p, S::CPU_HZ))); }
    if let Some(p) = &o.script { m.load_script(&std::fs::read_to_string(p).expect("script")).expect("script"); }
    if let Some(sec) = o.max_seconds { m.max_cycles = (sec * S::CPU_HZ as f64) as u64; }
    boot
}

/// The end-of-run report on stderr: counts, faults, peeks, observer reports, captures, registers.
fn report<S: Soc>(m: &mut Machine<S>, o: &Opts, stop: Stop, dt: f64) {
    let total = m.insns();
    let per: String = if S::CORES == 1 { format!("{}", total) } else { m.cores.iter().enumerate().map(|(i, c)| format!("core{} {}", i, c.insn_count())).collect::<Vec<_>>().join(" + ") };
    eprintln!("\n[emu] stop: {:?} — {} insns in {:.1}s wall = {:.1} Minsn/s; emulated {:.3}s ({} cycles); {} exceptions, {} interrupts",
              stop, per, dt, total as f64 / dt / 1e6, m.seconds(), m.bus.cycles(), m.exceptions, m.interrupts);
    if let Stop::Unimplemented(pc, raw) = stop {
        if let Ok(b) = m.bus.fetch(pc) { eprintln!("[emu] unimplemented at {:08x} {}: {} (raw {:#x})", pc, m.sym(pc), m.cores[0].disasm(pc, b), raw); }
    }
    if let Some((a, w)) = m.bus.last_fault() { eprintln!("[emu] last bus fault: {} {:#010x}", if w { "write" } else { "read" }, a); }
    for &(a, n) in &o.peeks { eprintln!("[peek after run]\n{}", m.peek(a, n)); }
    for &(a, n) in &o.disasms { eprintln!("[disasm {:#010x}]\n{}", a, m.disasm(a, n)); }
    { let r = m.reports(); if !r.is_empty() { eprintln!("{}", r); } }
    eprintln!("{}", m.irq_report());
    if let Some(w) = &o.wav { match m.write_wav(w) { Ok(n) => eprintln!("[emu] wrote {} samples ({:.2} s) to {}", n, n as f64 / m.bus.audio().1 as f64, w), Err(e) => eprintln!("[emu] wav: {}", e) } }
    { let r = m.bus.report(); if !r.is_empty() { eprintln!("{}", r); } }
    { let st: Vec<(u64, u64, u64, usize)> = m.cores.iter().filter_map(|c| c.code_cache_stats()).collect();
      if m.vq_stats[0] > 0 { eprintln!("[emu] virtual quanta: runs, quanta, stopped at a device register, at waiti = {:?}", m.vq_stats); }
      if st.iter().any(|s| s.0 > 0) { let b0 = st[0]; let b1 = st.get(1).copied().unwrap_or((0, 0, 0, 0)); eprintln!("[emu] blocks: {} built ({} cache flushes) core0, {} ({}) core1; jit: {} compiled, {} KB code", b0.0, b0.1, b1.0, b1.1, b0.2 + b1.2, (b0.3 + b1.3) / 1024); } }
    if m.stub_hits > 0 { eprintln!("[emu] stubs hit {} times", m.stub_hits); }
    if let Some(p) = &o.tft_png { match m.write_tft_png(p, 3) { Ok(()) => eprintln!("[emu] wrote {}", p), Err(e) => eprintln!("[emu] png: {}", e) } }
    if let Some(p) = &o.gram_png { match m.write_gram_png(p) { Ok(()) => eprintln!("[emu] wrote {}", p), Err(e) => eprintln!("[emu] png: {}", e) } }
    if o.dump { eprintln!("{}", m.dump_regs()); }
}

#[cfg(test)]
mod tests;
