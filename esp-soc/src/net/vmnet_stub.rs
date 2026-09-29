//! `--net bridge:PATH` where there are no Unix sockets (the WebAssembly build): the same
//! interface as `vmnet.rs`, and `connect` says why it cannot work here. No value of this type
//! can exist, so the other methods are never reached.

use super::BridgeStats;

pub const MAX_FRAME: usize = 64 * 1024;

pub struct Bridge { never: std::convert::Infallible }

impl Bridge {
    pub fn connect(path: &str, _mac: [u8; 6], _log: bool) -> Result<Self, String> {
        Err(format!("--net bridge:{path}: the bridge needs a Unix host with a socket_vmnet daemon"))
    }
    pub fn path(&self) -> &str { match self.never {} }
    pub fn station(&self) -> [u8; 6] { match self.never {} }
    pub fn send(&self, _eth: &[u8]) { match self.never {} }
    pub fn recv(&self, _max: usize) -> Vec<Vec<u8>> { match self.never {} }
    pub fn stats(&self) -> BridgeStats { match self.never {} }
}
