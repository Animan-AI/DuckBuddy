//! The Feetech bus, from the slave's side: framing, the register map, and the turn-taking a
//! `SYNC_READ` asks of every device it names.
//!
//! Pure logic, no hardware — so it builds and tests on a laptop:
//! `rustc --edition 2021 --test src/proto.rs -o target/proto-test && target/proto-test`.
//!
//! Frames are Dynamixel protocol 1.0 as Feetech uses it: `FF FF id len field params… chk`,
//! `len` = params + 2, `chk` = !(id + len + field + params). On the host's frames `field` is
//! the instruction; on a device's reply it is the error byte. The parser does not care which.

/// This board's bus ID. The host lists it last in every `SYNC_READ`, after the 15 servos.
pub const MY_ID: u8 = 200;
pub const BROADCAST: u8 = 0xFE;

pub const PING: u8 = 0x01;
pub const READ: u8 = 0x02;
pub const WRITE: u8 = 0x03;
pub const SYNC_READ: u8 = 0x82;

/// Where the host's tick reads, and how much: the servos' present-state block, 56..70.
pub const BLOCK_ADDR: u8 = 56;
pub const BLOCK_LEN: usize = 15;

/// Where the bus task's own counters can be read: 8 bytes, see `main.rs` (`DIAG`).
pub const DIAG_ADDR: u8 = 90;
pub const DIAG_LEN: usize = 8;

/// Largest parameter list accepted. A `SYNC_READ` naming 16 devices carries 18.
pub const MAX_PARAMS: usize = 40;
/// Largest read answered.
pub const MAX_READ: usize = 32;
/// A reply frame: header, id, len, error, data, checksum.
pub const MAX_FRAME: usize = MAX_READ + 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    pub id: u8,
    /// Instruction on a host frame, error byte on a device's reply.
    pub field: u8,
    pub params: [u8; MAX_PARAMS],
    pub nparams: usize,
}

impl Frame {
    pub fn params(&self) -> &[u8] {
        &self.params[..self.nparams]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Header1,
    Header2,
    Id,
    Len,
    Body,
}

/// Byte-at-a-time frame reassembly. Corrupt or oversized frames are dropped and the parser
/// goes back to hunting for `FF FF`, so a glitch costs one frame, never a desync.
pub struct Parser {
    state: State,
    id: u8,
    len: usize,
    body: [u8; MAX_PARAMS + 2],
    got: usize,
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    pub const fn new() -> Self {
        Self {
            state: State::Header1,
            id: 0,
            len: 0,
            body: [0; MAX_PARAMS + 2],
            got: 0,
        }
    }

    pub fn reset(&mut self) {
        self.state = State::Header1;
    }

    pub fn feed(&mut self, b: u8) -> Option<Frame> {
        match self.state {
            State::Header1 => {
                if b == 0xFF {
                    self.state = State::Header2;
                }
            }
            State::Header2 => {
                self.state = if b == 0xFF { State::Id } else { State::Header1 };
            }
            State::Id => {
                // A third FF is still header: some senders pad, and 0xFF is never an ID.
                if b != 0xFF {
                    self.id = b;
                    self.state = State::Len;
                }
            }
            State::Len => {
                let len = b as usize;
                if (2..=MAX_PARAMS + 2).contains(&len) {
                    self.len = len;
                    self.got = 0;
                    self.state = State::Body;
                } else {
                    self.state = State::Header1;
                }
            }
            State::Body => {
                self.body[self.got] = b;
                self.got += 1;
                if self.got == self.len {
                    self.state = State::Header1;
                    // body = field, params…, checksum
                    let sum = self.body[..self.len - 1]
                        .iter()
                        .fold(self.id.wrapping_add(self.len as u8), |a, x| a.wrapping_add(*x));
                    if !sum == self.body[self.len - 1] {
                        let nparams = self.len - 2;
                        let mut params = [0; MAX_PARAMS];
                        params[..nparams].copy_from_slice(&self.body[1..1 + nparams]);
                        return Some(Frame {
                            id: self.id,
                            field: self.body[0],
                            params,
                            nparams,
                        });
                    }
                }
            }
        }
        None
    }
}

/// What to put on the wire in answer to a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    Nothing,
    /// A status frame with no data — PING, and WRITE, which this board accepts and ignores.
    Ack,
    /// A status frame carrying `len` bytes of the register map from `addr`.
    Read { addr: u8, len: u8 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Pending {
    /// Our position in the `SYNC_READ` list: how many replies must pass before ours.
    index: usize,
    seen: usize,
    addr: u8,
    len: u8,
}

/// The slave's decisions, frame by frame.
///
/// A `SYNC_READ` is one request answered by every listed device in list order. This board
/// counts the replies ahead of it and speaks when the last has finished. The host lists it
/// last: Feetech servos start on their own timing rather than waiting for the device ahead,
/// so a servo listed after this board would talk over the end of its reply.
#[derive(Default)]
pub struct Slave {
    pending: Option<Pending>,
}

impl Slave {
    pub const fn new() -> Self {
        Self { pending: None }
    }

    /// Give up on a `SYNC_READ` turn that never came (a device ahead of us is silent).
    pub fn expire(&mut self) {
        self.pending = None;
    }

    pub fn waiting(&self) -> bool {
        self.pending.is_some()
    }

    pub fn on_frame(&mut self, f: &Frame) -> Action {
        if f.id == MY_ID {
            self.pending = None;
            return match f.field {
                PING | WRITE => Action::Ack,
                READ if f.nparams == 2 => Action::Read {
                    addr: f.params[0],
                    len: f.params[1],
                },
                // Includes our own reply heard back on the wire: its "field" is error 0.
                _ => Action::Nothing,
            };
        }
        if f.id == BROADCAST {
            self.pending = None;
            if f.field == SYNC_READ && f.nparams >= 3 {
                let p = f.params();
                if let Some(index) = p[2..].iter().position(|&id| id == MY_ID) {
                    let (addr, len) = (p[0], p[1]);
                    if index == 0 {
                        return Action::Read { addr, len };
                    }
                    self.pending = Some(Pending {
                        index,
                        seen: 0,
                        addr,
                        len,
                    });
                }
            }
            return Action::Nothing;
        }
        // Another device's reply: if we are queued behind it, count it.
        if let Some(mut p) = self.pending {
            p.seen += 1;
            if p.seen == p.index {
                self.pending = None;
                return Action::Read {
                    addr: p.addr,
                    len: p.len,
                };
            }
            self.pending = Some(p);
        }
        Action::Nothing
    }
}

/// The register map a host can read: identity at the bottom, the orientation block at 56.
pub fn read_map(block: &[u8; BLOCK_LEN], diag: &[u8; DIAG_LEN], addr: u8, out: &mut [u8]) {
    for (k, slot) in out.iter_mut().enumerate() {
        let a = addr as usize + k;
        *slot = match a {
            0 => 1, // firmware major
            1 => 0, // firmware minor
            3 => 0xD6, // "model": 0x16D6, chosen not to collide with any Feetech servo
            4 => 0x16,
            5 => MY_ID,
            6 => 1, // baud register: 500 kbit/s
            7 => 0, // return delay
            55 => 1, // "locked"
            _ if (BLOCK_ADDR as usize..BLOCK_ADDR as usize + BLOCK_LEN).contains(&a) => {
                block[a - BLOCK_ADDR as usize]
            }
            _ if (DIAG_ADDR as usize..DIAG_ADDR as usize + DIAG_LEN).contains(&a) => {
                diag[a - DIAG_ADDR as usize]
            }
            _ => 0,
        };
    }
}

/// Build a status frame (error 0) around `data`. Returns the frame length.
pub fn status_frame(data: &[u8], out: &mut [u8; MAX_FRAME]) -> usize {
    let n = data.len().min(MAX_READ);
    out[0] = 0xFF;
    out[1] = 0xFF;
    out[2] = MY_ID;
    out[3] = (n + 2) as u8;
    out[4] = 0;
    out[5..5 + n].copy_from_slice(&data[..n]);
    let sum = out[2..5 + n].iter().fold(0u8, |a, x| a.wrapping_add(*x));
    out[5 + n] = !sum;
    n + 6
}

/// Answer an action: fill `out` with the reply frame, or return 0 for silence.
pub fn reply(
    action: Action,
    block: &[u8; BLOCK_LEN],
    diag: &[u8; DIAG_LEN],
    out: &mut [u8; MAX_FRAME],
) -> usize {
    match action {
        Action::Nothing => 0,
        Action::Ack => status_frame(&[], out),
        Action::Read { addr, len } => {
            let len = (len as usize).min(MAX_READ);
            let mut data = [0u8; MAX_READ];
            read_map(block, diag, addr, &mut data[..len]);
            status_frame(&data[..len], out)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(id: u8, field: u8, params: &[u8]) -> ([u8; 64], usize) {
        let mut b = [0u8; 64];
        b[0] = 0xFF;
        b[1] = 0xFF;
        b[2] = id;
        b[3] = (params.len() + 2) as u8;
        b[4] = field;
        b[5..5 + params.len()].copy_from_slice(params);
        let sum = b[2..5 + params.len()].iter().fold(0u8, |a, x| a.wrapping_add(*x));
        b[5 + params.len()] = !sum;
        (b, params.len() + 6)
    }

    fn parse_all(bytes: &[u8]) -> Option<Frame> {
        let mut p = Parser::new();
        let mut last = None;
        for &b in bytes {
            if let Some(f) = p.feed(b) {
                last = Some(f);
            }
        }
        last
    }

    /// The checksum example from the protocol spec, `FF FF FE 06 82 1E 02 0B 0C 42`.
    #[test]
    fn parses_the_spec_example() {
        let f = parse_all(&[0xFF, 0xFF, 0xFE, 0x06, 0x82, 0x1E, 0x02, 0x0B, 0x0C, 0x42]).unwrap();
        assert_eq!((f.id, f.field), (0xFE, 0x82));
        assert_eq!(f.params(), &[0x1E, 0x02, 0x0B, 0x0C]);
    }

    #[test]
    fn a_bad_checksum_is_dropped_and_the_next_frame_still_parses() {
        let (mut a, na) = frame(MY_ID, PING, &[]);
        a[na - 1] ^= 1;
        let (b, nb) = frame(MY_ID, READ, &[56, 15]);
        let mut bytes = [0u8; 128];
        bytes[..na].copy_from_slice(&a[..na]);
        bytes[na..na + nb].copy_from_slice(&b[..nb]);
        let mut p = Parser::new();
        let frames: usize = bytes[..na + nb].iter().filter_map(|&x| p.feed(x)).count();
        assert_eq!(frames, 1);
    }

    #[test]
    fn leading_garbage_and_a_third_ff_are_tolerated() {
        let (f, n) = frame(MY_ID, PING, &[]);
        let mut bytes = [0u8; 32];
        bytes[..4].copy_from_slice(&[0x00, 0x13, 0xFF, 0x7E]);
        bytes[4] = 0xFF;
        bytes[5..5 + n].copy_from_slice(&f[..n]);
        assert_eq!(parse_all(&bytes[..5 + n]).unwrap().field, PING);
    }

    #[test]
    fn first_in_a_sync_read_answers_at_once() {
        let (b, n) = frame(BROADCAST, SYNC_READ, &[56, 15, MY_ID, 8, 11, 6]);
        let f = parse_all(&b[..n]).unwrap();
        let mut s = Slave::new();
        assert_eq!(s.on_frame(&f), Action::Read { addr: 56, len: 15 });
    }

    /// Listed third, it must let two replies pass before speaking.
    #[test]
    fn later_in_a_sync_read_waits_its_turn() {
        let (b, n) = frame(BROADCAST, SYNC_READ, &[56, 15, 8, 11, MY_ID]);
        let mut s = Slave::new();
        assert_eq!(s.on_frame(&parse_all(&b[..n]).unwrap()), Action::Nothing);
        let (r1, n1) = frame(8, 0, &[0; 15]);
        assert_eq!(s.on_frame(&parse_all(&r1[..n1]).unwrap()), Action::Nothing);
        let (r2, n2) = frame(11, 0, &[0; 15]);
        assert_eq!(
            s.on_frame(&parse_all(&r2[..n2]).unwrap()),
            Action::Read { addr: 56, len: 15 }
        );
        assert!(!s.waiting());
    }

    #[test]
    fn a_sync_read_without_us_is_ignored() {
        let (b, n) = frame(BROADCAST, SYNC_READ, &[56, 15, 1, 2, 3]);
        let mut s = Slave::new();
        assert_eq!(s.on_frame(&parse_all(&b[..n]).unwrap()), Action::Nothing);
        assert!(!s.waiting());
    }

    /// Our own reply, heard back on the shared wire, must never trigger another reply.
    #[test]
    fn our_own_echo_is_not_an_instruction() {
        let mut out = [0u8; MAX_FRAME];
        let block = [7u8; BLOCK_LEN];
        let n = reply(Action::Read { addr: 56, len: 15 }, &block, &[0; DIAG_LEN], &mut out);
        let f = parse_all(&out[..n]).unwrap();
        assert_eq!(f.id, MY_ID);
        assert_eq!(Slave::new().on_frame(&f), Action::Nothing);
    }

    #[test]
    fn ping_and_read_are_answered() {
        let mut s = Slave::new();
        let (b, n) = frame(MY_ID, PING, &[]);
        assert_eq!(s.on_frame(&parse_all(&b[..n]).unwrap()), Action::Ack);
        let (b, n) = frame(MY_ID, READ, &[5, 3]);
        assert_eq!(
            s.on_frame(&parse_all(&b[..n]).unwrap()),
            Action::Read { addr: 5, len: 3 }
        );
    }

    #[test]
    fn the_map_serves_identity_and_the_block() {
        let mut block = [0u8; BLOCK_LEN];
        for (i, b) in block.iter_mut().enumerate() {
            *b = 100 + i as u8;
        }
        let mut out = [0u8; 4];
        read_map(&block, &[0; DIAG_LEN], 5, &mut out);
        assert_eq!(out, [MY_ID, 1, 0, 0]);
        let mut out = [0u8; BLOCK_LEN];
        read_map(&block, &[0; DIAG_LEN], BLOCK_ADDR, &mut out);
        let diag = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut d = [0u8; DIAG_LEN];
        read_map(&block, &diag, DIAG_ADDR, &mut d);
        assert_eq!(d, diag);
        assert_eq!(out, block);
    }

    /// The reply must be a frame the host's own parser accepts — checked by parsing it back.
    #[test]
    fn replies_round_trip_through_the_parser() {
        let block = [0x5Au8; BLOCK_LEN];
        let mut out = [0u8; MAX_FRAME];
        let n = reply(Action::Read { addr: BLOCK_ADDR, len: 15 }, &block, &[0; DIAG_LEN], &mut out);
        assert_eq!(n, 21);
        let f = parse_all(&out[..n]).unwrap();
        assert_eq!((f.id, f.field), (MY_ID, 0));
        assert_eq!(f.params(), &block);
        assert_eq!(reply(Action::Ack, &block, &[0; DIAG_LEN], &mut out), 6);
        assert_eq!(&out[..6], &[0xFF, 0xFF, MY_ID, 2, 0, !(MY_ID + 2)]);
    }
}
