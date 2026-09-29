//! Minimal RGB8 PNG writer: stored (uncompressed) deflate blocks, no zlib dependency.
//! Same approach as `esp-soc/src/png.rs`, which only takes RGB565.

fn crc32(data: &[u8]) -> u32 {
    let mut c = 0xffff_ffffu32;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
        }
    }
    !c
}

fn adler32(data: &[u8]) -> u32 {
    let (mut a, mut b) = (1u32, 0u32);
    for &d in data {
        a = (a + d as u32) % 65521;
        b = (b + a) % 65521;
    }
    (b << 16) | a
}

fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    let start = out.len();
    out.extend_from_slice(kind);
    out.extend_from_slice(data);
    let crc = crc32(&out[start..]);
    out.extend_from_slice(&crc.to_be_bytes());
}

/// Encode `rgb` (row-major, 3 bytes per pixel) as a PNG file image.
pub fn encode_rgb8(rgb: &[u8], width: usize, height: usize) -> Vec<u8> {
    assert_eq!(rgb.len(), width * height * 3);
    let mut raw = Vec::with_capacity(height * (width * 3 + 1));
    for row in rgb.chunks_exact(width * 3) {
        raw.push(0); // filter: none
        raw.extend_from_slice(row);
    }
    let mut z = vec![0x78, 0x01];
    let blocks = raw.chunks(65535).count().max(1);
    for (i, blk) in raw.chunks(65535).enumerate() {
        z.push((i + 1 == blocks) as u8);
        z.extend_from_slice(&(blk.len() as u16).to_le_bytes());
        z.extend_from_slice(&(!(blk.len() as u16)).to_le_bytes());
        z.extend_from_slice(blk);
    }
    z.extend_from_slice(&adler32(&raw).to_be_bytes());
    let mut out = vec![137, 80, 78, 71, 13, 10, 26, 10];
    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&(width as u32).to_be_bytes());
    ihdr.extend_from_slice(&(height as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]); // 8-bit, truecolour, deflate, no filter, no interlace
    chunk(&mut out, b"IHDR", &ihdr);
    chunk(&mut out, b"IDAT", &z);
    chunk(&mut out, b"IEND", &[]);
    out
}

pub fn write_rgb8(path: impl AsRef<std::path::Path>, rgb: &[u8], width: usize, height: usize) -> std::io::Result<()> {
    std::fs::write(path, encode_rgb8(rgb, width, height))
}

#[cfg(test)]
mod tests {
    #[test]
    fn crc_of_iend_is_the_well_known_value() {
        // Every PNG ends in IEND with CRC AE 42 60 82 (CRC-32 of the four bytes "IEND").
        assert_eq!(super::crc32(b"IEND"), 0xAE42_6082);
        let png = super::encode_rgb8(&[255, 0, 0, 0, 255, 0], 2, 1);
        assert_eq!(&png[..8], &[137, 80, 78, 71, 13, 10, 26, 10]);
        assert_eq!(&png[png.len() - 4..], &[0xAE, 0x42, 0x60, 0x82]);
    }
}
