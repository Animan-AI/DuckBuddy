//! Duck-speak: a sentence said in this duck's own voice, one quack per syllable.
//!
//! The robot's answers come from a language model as Chinese text; a synthetic human voice
//! saying them sounded wrong on a duck whatever was done to it (2026-10-15: four cloned and
//! re-voiced takes, none of them a duck). So the duck *speaks duck*: every syllable of the
//! answer is one call of the voice the quacks are made of ([`Stream::speaking`] — same
//! harmonics, nasality, formant and buzz), and what survives of the language is what carries
//! a sentence's feeling in Mandarin anyway:
//!
//! - **the rhythm** — one call per character, shorter on a neutral tone, longer before a pause;
//! - **the tones** — each syllable's pitch draws its tone: level, rising, dipping, falling;
//! - **the vowel** — `a` / `o` / `e` / `i` / `u` open the beak and move the formant as the
//!   chorale's sung vowels do ([`Vowel`]), so the words have colour, and the beak shows it;
//! - **the punctuation** — commas breathe, a question lifts its last syllable, an
//!   exclamation is higher and louder;
//! - **the mood** — the answer's emotion tag sets register, tempo, how far the tones swing,
//!   loudness and drawl ([`Mood::named`]).
//!
//! A run of three or more laughing syllables (`ha1 ha1 ha1`, 嘿嘿嘿, 嘻嘻嘻, 嘎嘎嘎) is not read
//! out syllable by syllable but *laughed*: a longer string of short, breathy bursts that tumble
//! down in pitch and end in a wheeze ([`Token::Laugh`]) — the user found the duck's three quick
//! "哈" funny and wanted more of it (2026-10-15).
//!
//! Input is pinyin with tone numbers (`kai1 xin1 ya5 !`), written by whoever has the text
//! (the brain, `tools/duckbrain`) — the synth needs only the tone and the vowel of each.

use anyhow::{Result, bail};

use crate::chorale::Vowel;
use crate::personality::Personality;
use crate::stream::Stream;
use crate::synth::SR;

/// One unit of a sentence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Token {
    /// A syllable: its tone (1-4, 0 neutral), its vowel, and whether it starts hard (a plosive
    /// initial: b p d t g k) — a sharper attack and a bigger onset blip, the "k" in a quack.
    Syllable {
        tone: u8,
        vowel: Vowel,
        hard: bool,
    },
    /// A laugh, from a run of `run` laughing syllables: more bursts than that, shorter.
    Laugh {
        run: u32,
    },
    Pause(Pause),
}

/// Syllables that laugh when three or more of them come in a row.
const LAUGHS: [&str; 7] = ["ha", "he", "hei", "hi", "xi", "ga", "gua"];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pause {
    Comma,
    Stop,
    Question,
    Exclaim,
}

impl Pause {
    fn seconds(&self) -> f64 {
        match self {
            Pause::Comma => 0.16,
            Pause::Stop => 0.32,
            Pause::Question => 0.32,
            Pause::Exclaim => 0.28,
        }
    }
}

/// How a sentence is said: the emotion tag, as prosody.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mood {
    /// Register, semitones from the duck's own centre.
    pub pitch_st: f64,
    /// Speaking rate, 1 = normal.
    pub tempo: f64,
    /// How far the tones swing, 1 = normal.
    pub spread: f64,
    /// Loudness, 0..1.
    pub level: f64,
    /// Extra length on phrase-final syllables, as a fraction (the coy drawl).
    pub drawl: f64,
}

impl Default for Mood {
    fn default() -> Self {
        Self {
            pitch_st: 0.0,
            tempo: 1.0,
            spread: 1.0,
            level: 0.9,
            drawl: 0.0,
        }
    }
}

impl Mood {
    /// The brain's emotion tags (English or the Chinese the language model writes).
    pub fn named(name: &str) -> Option<Self> {
        let m = |pitch_st, tempo, spread, level, drawl| {
            Some(Self {
                pitch_st,
                tempo,
                spread,
                level,
                drawl,
            })
        };
        match name {
            "neutral" | "平静" => Some(Self::default()),
            "happy" | "开心" => m(2.0, 1.12, 1.2, 1.0, 0.0),
            "curious" | "好奇" => m(1.0, 1.0, 1.15, 0.9, 0.1),
            "coy" | "撒娇" => m(3.0, 0.85, 1.3, 0.85, 0.6),
            "shy" | "害羞" => m(1.0, 0.9, 0.8, 0.65, 0.2),
            "sleepy" | "困" => m(-3.0, 0.72, 0.6, 0.6, 0.4),
            "sad" | "难过" => m(-2.0, 0.8, 0.7, 0.7, 0.3),
            "surprised" | "惊讶" => m(4.0, 1.2, 1.5, 1.0, 0.0),
            _ => None,
        }
    }
}

/// Parse `kai1 xin1 ya5 ， zhu3 ren2 ？` — pinyin syllables with a tone digit (5 or none is
/// neutral), and punctuation (ASCII or full-width) as its own word or stuck to a syllable.
pub fn parse(spec: &str) -> Result<Vec<Token>> {
    let mut out = Vec::new();
    // the bare syllable of each token, for spotting a laugh
    let mut bare: Vec<Option<String>> = Vec::new();
    for word in spec.split_whitespace() {
        let mut syllable = String::new();
        for c in word.chars() {
            if let Some(p) = pause_of(c) {
                if !syllable.is_empty() {
                    out.push(syllable_token(&syllable)?);
                    bare.push(Some(bare_of(&syllable)));
                    syllable.clear();
                }
                // a run of marks ("？！", "……") is one pause, the strongest wins
                match out.last_mut() {
                    Some(Token::Pause(prev)) => {
                        if p != Pause::Comma {
                            *prev = p;
                        }
                    }
                    _ => {
                        out.push(Token::Pause(p));
                        bare.push(None);
                    }
                }
            } else {
                syllable.push(c);
            }
        }
        if !syllable.is_empty() {
            out.push(syllable_token(&syllable)?);
            bare.push(Some(bare_of(&syllable)));
        }
    }
    // three or more laughing syllables in a row are a laugh
    let mut laughed = Vec::with_capacity(out.len());
    let mut i = 0;
    while i < out.len() {
        let laughs = |k: usize| bare[k].as_deref().is_some_and(|b| LAUGHS.contains(&b));
        let mut j = i;
        while j < out.len() && laughs(j) && bare[j] == bare[i] {
            j += 1;
        }
        if j - i >= 3 {
            laughed.push(Token::Laugh {
                run: (j - i) as u32,
            });
            i = j;
        } else {
            laughed.push(out[i]);
            i += 1;
        }
    }
    Ok(laughed)
}

fn bare_of(s: &str) -> String {
    s.to_lowercase()
        .trim_end_matches(|c: char| c.is_ascii_digit())
        .to_string()
}

fn pause_of(c: char) -> Option<Pause> {
    match c {
        ',' | '，' | '、' | ';' | '；' | ':' | '：' | '…' | '~' | '～' | '—' => {
            Some(Pause::Comma)
        }
        '.' | '。' => Some(Pause::Stop),
        '?' | '？' => Some(Pause::Question),
        '!' | '！' => Some(Pause::Exclaim),
        _ => None,
    }
}

fn syllable_token(s: &str) -> Result<Token> {
    let s = s.to_lowercase();
    let (body, tone) = match s.chars().last() {
        Some(d @ '0'..='5') => (&s[..s.len() - 1], (d as u8 - b'0') % 5),
        _ => (s.as_str(), 0),
    };
    if body.is_empty() || !body.chars().all(|c| c.is_ascii_lowercase() || c == 'ü') {
        bail!("not a pinyin syllable: {s:?}");
    }
    let hard = matches!(body.chars().next(), Some('b' | 'p' | 'd' | 't' | 'g' | 'k'));
    Ok(Token::Syllable {
        tone,
        vowel: vowel_of(body),
        hard,
    })
}

/// The vowel a syllable is heard on: its final's nucleus, as one of the chorale's six.
fn vowel_of(body: &str) -> Vowel {
    let initials = [
        "zh", "ch", "sh", "b", "p", "m", "f", "d", "t", "n", "l", "g", "k", "h", "j", "q", "x",
        "r", "z", "c", "s", "y", "w",
    ];
    let initial = initials
        .iter()
        .find(|i| body.starts_with(**i))
        .copied()
        .unwrap_or("");
    let fin = &body[initial.len()..];
    // j q x y + u is ü
    let umlaut = matches!(initial, "j" | "q" | "x" | "y") && fin.starts_with('u');
    if fin.is_empty() {
        // m, n, ng, hm: a hum (and "zh" alone is not pinyin, but hum it rather than fail)
        return Vowel::Mm;
    }
    if fin.contains('ü') || fin.contains('v') || umlaut {
        return Vowel::Ee;
    }
    if fin.starts_with("ian") || fin == "yan" {
        return Vowel::Eh;
    }
    if fin.contains('a') {
        return Vowel::Ah;
    }
    if fin.contains("ong") {
        return Vowel::Oo;
    }
    if fin.contains('o') {
        return Vowel::Oh;
    }
    if fin.contains('e') {
        return Vowel::Eh;
    }
    if fin.contains('i') {
        return Vowel::Ee;
    }
    if fin.contains('u') {
        return Vowel::Oo;
    }
    Vowel::Mm
}

/// Base syllable length, seconds, at tempo 1.
const SYLLABLE_S: f64 = 0.17;
/// How far the pitch drifts down over a phrase, semitones per second (declination).
const DECLINE_ST_PER_S: f64 = 1.2;

/// A tone's pitch, semitones from the phrase line, `u` 0..1 through the syllable.
fn tone_st(tone: u8, u: f64, final_in_phrase: bool) -> f64 {
    let smooth = u * u * (3.0 - 2.0 * u);
    match tone {
        1 => 2.5,
        2 => -1.0 + 4.0 * smooth,
        // the third tone dips; only phrase-final does it come back up
        3 if final_in_phrase => {
            if u < 0.6 {
                -1.0 - 3.5 * (u / 0.6)
            } else {
                -4.5 + 3.0 * ((u - 0.6) / 0.4)
            }
        }
        3 => -1.0 - 3.5 * smooth,
        4 => 3.5 - 6.5 * u.powf(0.8),
        _ => -1.5,
    }
}

/// Render a sentence: the samples (at [`SR`]) and the beak opening per 20 ms frame (0..1),
/// which is what the mouth servo should be sent while it plays.
pub fn render(
    p: &Personality,
    tokens: &[Token],
    mood: &Mood,
    variant: u32,
) -> (Vec<f32>, Vec<f32>) {
    struct Seg {
        start: f64,
        len: f64,
        /// A laugh burst: its place in the laugh, 0..1 (pitch tumbles down it), and breathier.
        laugh: Option<f64>,
        tone: u8,
        vowel: Vowel,
        hard: bool,
        final_in_phrase: bool,
        lift_end_st: f64,
        phrase_start: f64,
        loud: f64,
    }
    // Lay the sentence out in time.
    let mut segs: Vec<Seg> = Vec::new();
    let mut t = 0.08;
    let mut phrase_start = t;
    let mut i = 0;
    while i < tokens.len() {
        match tokens[i] {
            Token::Syllable { tone, vowel, hard } => {
                let next_pause = tokens.get(i + 1).map(|x| match x {
                    Token::Pause(p) => Some(*p),
                    Token::Syllable { .. } | Token::Laugh { .. } => None,
                });
                let final_in_phrase = !matches!(next_pause, Some(None));
                let pause = next_pause.flatten();
                let mut len = SYLLABLE_S / mood.tempo;
                if tone == 0 {
                    len *= 0.65;
                }
                if final_in_phrase {
                    len *= 1.35 + mood.drawl;
                }
                let lift_end_st = if final_in_phrase && pause == Some(Pause::Question) {
                    4.0
                } else {
                    0.0
                };
                let loud = if pause == Some(Pause::Exclaim) {
                    1.0
                } else {
                    0.9
                };
                segs.push(Seg {
                    start: t,
                    len,
                    laugh: None,
                    tone,
                    vowel,
                    hard,
                    final_in_phrase,
                    lift_end_st,
                    phrase_start,
                    loud,
                });
                t += len;
            }
            Token::Laugh { run } => {
                // more bursts than were written, quicker than speech, a little irregular, the
                // last one drawn out into a wheeze
                let bursts = (2 * run + 3).clamp(7, 12) as usize;
                let mut rng = p.variant_rng("laugh", variant + run);
                for b in 0..bursts {
                    let last = b + 1 == bursts;
                    let len = if last {
                        0.34
                    } else {
                        (0.105 + 0.03 * rng.random()) / mood.tempo.sqrt()
                    };
                    let place = b as f64 / (bursts - 1) as f64;
                    segs.push(Seg {
                        start: t,
                        len,
                        laugh: Some(place),
                        tone: if last { 4 } else { 1 },
                        vowel: Vowel::Ah,
                        hard: true,
                        final_in_phrase: last,
                        lift_end_st: 0.0,
                        phrase_start,
                        loud: 1.0,
                    });
                    t += len
                        + if last {
                            0.0
                        } else {
                            0.035 + 0.02 * rng.random()
                        };
                }
                phrase_start = t;
            }
            Token::Pause(p) => {
                t += p.seconds() / mood.tempo.sqrt();
                phrase_start = t;
            }
        }
        i += 1;
    }
    let total_s = t + 0.15;
    let total = (total_s * f64::from(SR)) as usize;

    let mut stream = Stream::speaking(p, variant);
    stream.set_speaker_rolloff(Some(300.0));
    let centre = p.pitch_center_hz * 2f64.powf(mood.pitch_st / 12.0);
    // how far this duck's tones swing: its own pitch spread, shaped by the mood
    let swing = mood.spread * (0.7 + 0.3 * p.pitch_spread.clamp(0.0, 1.5));
    const BLOCK: usize = 96; // 2 ms
    let mut out = vec![0.0f32; total];
    let frames = (total_s / 0.02).ceil() as usize;
    let mut mouth = vec![0.0f32; frames];
    let mut k = 0usize;
    let mut last_hz = centre;
    for start in (0..total).step_by(BLOCK) {
        let now = start as f64 / f64::from(SR);
        while k < segs.len() && now >= segs[k].start + segs[k].len {
            k += 1;
        }
        let (hz, level, open, formant) = match segs.get(k) {
            Some(s) if now >= s.start => {
                let u = ((now - s.start) / s.len).clamp(0.0, 1.0);
                let into = now - s.start;
                let decline = -DECLINE_ST_PER_S * (s.start - s.phrase_start);
                let blip = if s.hard { 2.5 } else { 1.2 } * (-into / 0.025).exp();
                let st = match s.laugh {
                    // a laugh starts high and tumbles down; each burst a quick up-and-down
                    Some(place) => {
                        5.0 - 7.0 * place + 2.0 * (std::f64::consts::PI * u).sin() + blip
                    }
                    None => {
                        decline
                            + swing * tone_st(s.tone, u, s.final_in_phrase)
                            + s.lift_end_st * u * u
                            + blip
                            + if s.loud >= 1.0 { 1.5 } else { 0.0 }
                    }
                };
                let hz = centre * 2f64.powf(st / 12.0);
                // the call's envelope, a quack's: a quick attack onto an accent that sags through
                // the syllable (the greet's exponential decay, gentler), and a dip at the end so
                // syllables separate
                let attack = if s.hard { 0.010 } else { 0.022 };
                let accent = 1.0 - 0.3 * u;
                let tail = if u > 0.8 {
                    1.0 - (u - 0.8) / 0.2 * 0.75
                } else {
                    1.0
                };
                let env = (into / attack).min(1.0) * accent * tail;
                // the beak opens wider than the chorale's stage vowels at speaking speed, and the
                // vowel moves the formant only half as far: a sung `oo` two harmonics down from the
                // duck's own formant is a hollow hoot, and a sentence is mostly vowels
                let open = (0.3 + (s.vowel.open().max(0.5) - 0.3) * env).clamp(0.0, 1.0);
                last_hz = hz;
                (
                    hz,
                    mood.level * s.loud * s.vowel.level_scale() * env,
                    open,
                    0.5 * s.vowel.formant_shift(),
                )
            }
            _ => (last_hz, 0.0, 0.05, 0.0),
        };
        stream.set_formant_shift(formant);
        stream.set(hz, level, open);
        let end = (start + BLOCK).min(total);
        stream.block(&mut out[start..end]);
        let f = (now / 0.02) as usize;
        if f < frames {
            mouth[f] = mouth[f].max(if level > 0.05 { open as f32 } else { 0.0 });
        }
    }
    (out, mouth)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pinyin_reads_as_tones_vowels_and_pauses() {
        let t = parse("kai1 xin1 ya5 ！zhu3 ren2，ni3 hao3 ma？").unwrap();
        assert_eq!(
            t[0],
            Token::Syllable {
                tone: 1,
                vowel: Vowel::Ah,
                hard: true
            }
        );
        assert_eq!(
            t[1],
            Token::Syllable {
                tone: 1,
                vowel: Vowel::Ee,
                hard: false
            }
        );
        assert_eq!(
            t[2],
            Token::Syllable {
                tone: 0,
                vowel: Vowel::Ah,
                hard: false
            }
        );
        assert_eq!(t[3], Token::Pause(Pause::Exclaim));
        assert_eq!(t[6], Token::Pause(Pause::Comma));
        assert_eq!(*t.last().unwrap(), Token::Pause(Pause::Question));
        // ü after j q x y, -ian is "ien", -ong is dark
        assert_eq!(
            parse("qu4").unwrap()[0],
            Token::Syllable {
                tone: 4,
                vowel: Vowel::Ee,
                hard: false
            }
        );
        assert_eq!(
            parse("tian1").unwrap()[0],
            Token::Syllable {
                tone: 1,
                vowel: Vowel::Eh,
                hard: true
            }
        );
        assert_eq!(
            parse("dong4").unwrap()[0],
            Token::Syllable {
                tone: 4,
                vowel: Vowel::Oo,
                hard: true
            }
        );
        assert!(parse("ni3 h@o").is_err());
    }

    #[test]
    fn three_laughing_syllables_are_a_laugh_and_it_lasts() {
        let t = parse("ha1 ha1 ha1 ！ hao3 ya5").unwrap();
        assert_eq!(t[0], Token::Laugh { run: 3 });
        assert_eq!(t[1], Token::Pause(Pause::Exclaim));
        assert_eq!(t.len(), 4);
        // two is not a laugh, and a run must be the same syllable
        assert!(
            parse("ga1 ga1 ！")
                .unwrap()
                .iter()
                .all(|x| !matches!(x, Token::Laugh { .. }))
        );
        assert!(
            parse("ha1 he1 hi1")
                .unwrap()
                .iter()
                .all(|x| !matches!(x, Token::Laugh { .. }))
        );
        let p = Personality::from_seed(3_764_158_588);
        let (laugh, _) = render(
            &p,
            &parse("ha1 ha1 ha1").unwrap(),
            &Mood::named("开心").unwrap(),
            0,
        );
        let (said, _) = render(
            &p,
            &parse("ni3 hao3 ya5").unwrap(),
            &Mood::named("开心").unwrap(),
            0,
        );
        let secs = laugh.len() as f64 / f64::from(SR);
        assert!(
            secs > 1.2 && laugh.len() > 2 * said.len(),
            "a laugh of {secs} s"
        );
    }

    #[test]
    fn a_sentence_takes_its_syllables_time_and_the_beak_moves_with_it() {
        let p = Personality::from_seed(3_764_158_588);
        let t = parse("ni3 hao3 ， xiao3 bai2 ！").unwrap();
        let (wav, mouth) = render(&p, &t, &Mood::default(), 0);
        let secs = wav.len() as f64 / f64::from(SR);
        assert!((0.9..2.0).contains(&secs), "{secs} s");
        assert!(wav.iter().all(|x| x.is_finite() && x.abs() <= 1.0));
        // the beak opens on the syllables and closes in the comma
        assert!(mouth.iter().any(|&m| m > 0.5));
        assert!(mouth.iter().filter(|&&m| m == 0.0).count() > 5);
        // a sad sentence is slower than a happy one
        let slow = render(&p, &t, &Mood::named("难过").unwrap(), 0).0.len();
        let fast = render(&p, &t, &Mood::named("开心").unwrap(), 0).0.len();
        assert!(slow > fast);
    }
}
