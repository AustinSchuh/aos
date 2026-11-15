//! Matching each camera frame with the overlay for it, by capture time.
//!
//! The detector sends what it found in a frame (its `TargetMap`) some milliseconds after the
//! frame, stamped with the frame's capture time.  So each frame waits, ready to encode, until the
//! overlay for exactly its capture time arrives, or until a deadline, and goes out with whatever
//! it got.  Frames always go out in capture order.  Overlays for frames that were never queued
//! (skipped to keep to the stream's frame rate) are dropped, and nothing is drawn on a frame it
//! wasn't made for.
//!
//! A camera whose detector has said nothing for a while isn't waited for, so a stopped detector
//! costs no latency; its next message brings the wait back.

use std::collections::VecDeque;

use crate::overlay::Overlay;

/// How long frames wait, and when a detector counts as gone.
#[derive(Clone, Copy, Debug)]
pub struct JoinOptions {
    /// The longest a frame waits for its overlay.
    pub max_wait_ns: i64,
    /// A detector silent for this long isn't waited for.
    pub absent_after_ns: i64,
}

/// A frame and what to draw on it, ready to encode.
#[derive(Debug)]
pub struct Ready<F> {
    pub frame: F,
    pub capture_ns: i64,
    /// `None` if its overlay didn't arrive in time, or there's no detector.
    pub overlay: Option<Overlay>,
}

struct Waiting<F> {
    frame: F,
    capture_ns: i64,
    deadline_ns: i64,
}

/// One camera's frames waiting for their overlays.
pub struct Join<F> {
    options: JoinOptions,
    waiting: VecDeque<Waiting<F>>,
    last_overlay_ns: Option<i64>,
    /// Counts, for the status channel.
    /// Frames sent with their overlay.
    pub drawn: u64,
    /// Frames sent without, their overlay not having come by their deadline.
    pub missed: u64,
    /// Overlays for no waiting frame: one skipped for the stream's frame rate, or already sent
    /// at its deadline.
    pub unmatched: u64,
}

impl<F> Join<F> {
    pub fn new(options: JoinOptions) -> Self {
        Self {
            options,
            waiting: VecDeque::new(),
            last_overlay_ns: None,
            drawn: 0,
            missed: 0,
            unmatched: 0,
        }
    }

    fn detector_present(&self, now_ns: i64) -> bool {
        self.last_overlay_ns
            .is_some_and(|last| now_ns - last < self.options.absent_after_ns)
    }

    /// A new frame, captured at `capture_ns`, arriving at `now_ns`.  Returns the frames now ready.
    pub fn add_frame(&mut self, frame: F, capture_ns: i64, now_ns: i64) -> Vec<Ready<F>> {
        if !self.detector_present(now_ns) {
            // Nothing to wait for; don't hold this frame behind earlier ones either.
            let mut ready = self.flush_all();
            ready.push(Ready {
                frame,
                capture_ns,
                overlay: None,
            });
            return ready;
        }
        self.waiting.push_back(Waiting {
            frame,
            capture_ns,
            deadline_ns: now_ns + self.options.max_wait_ns,
        });
        Vec::new()
    }

    /// An overlay arriving at `now_ns`.  Returns the frames now ready.
    pub fn add_overlay(&mut self, overlay: Overlay, now_ns: i64) -> Vec<Ready<F>> {
        self.last_overlay_ns = Some(now_ns);
        let stamp = overlay.capture_ns;
        let mut ready = Vec::new();
        // Anything captured before this one has missed its overlay: it would have come first.
        while self.waiting.front().is_some_and(|w| w.capture_ns < stamp) {
            self.missed += 1;
            ready.push(self.pop(None));
        }
        if self.waiting.front().is_some_and(|w| w.capture_ns == stamp) {
            self.drawn += 1;
            ready.push(self.pop(Some(overlay)));
        } else {
            self.unmatched += 1;
        }
        ready
    }

    /// Frames whose deadline has passed by `now_ns`, without their overlays.
    pub fn expire(&mut self, now_ns: i64) -> Vec<Ready<F>> {
        let mut ready = Vec::new();
        while self
            .waiting
            .front()
            .is_some_and(|w| w.deadline_ns <= now_ns)
        {
            self.missed += 1;
            ready.push(self.pop(None));
        }
        ready
    }

    /// When the oldest waiting frame's deadline is, if any frame is waiting.
    pub fn next_deadline_ns(&self) -> Option<i64> {
        self.waiting.front().map(|w| w.deadline_ns)
    }

    fn pop(&mut self, overlay: Option<Overlay>) -> Ready<F> {
        let waiting = self.waiting.pop_front().expect("Checked before popping");
        Ready {
            frame: waiting.frame,
            capture_ns: waiting.capture_ns,
            overlay,
        }
    }

    fn flush_all(&mut self) -> Vec<Ready<F>> {
        let mut ready = Vec::new();
        while !self.waiting.is_empty() {
            self.missed += 1;
            ready.push(self.pop(None));
        }
        ready
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: i64 = 1_000_000;
    const OPTIONS: JoinOptions = JoinOptions {
        max_wait_ns: 50 * MS,
        absent_after_ns: 1000 * MS,
    };

    fn stamped(capture_ns: i64) -> Overlay {
        Overlay {
            capture_ns,
            lines: vec![],
        }
    }

    fn captures(ready: &[Ready<&str>]) -> Vec<(i64, bool)> {
        ready
            .iter()
            .map(|r| (r.capture_ns, r.overlay.is_some()))
            .collect()
    }

    /// A detector that has spoken, so frames wait for it.
    fn with_detector() -> Join<&'static str> {
        let mut join = Join::new(OPTIONS);
        assert!(join.add_overlay(stamped(0), 0).is_empty());
        join
    }

    #[test]
    fn without_a_detector_frames_go_straight_out() {
        let mut join = Join::new(OPTIONS);
        let ready = join.add_frame("a", 100 * MS, 101 * MS);
        assert_eq!(captures(&ready), vec![(100 * MS, false)]);
        assert_eq!(join.next_deadline_ns(), None);
    }

    #[test]
    fn each_frame_gets_exactly_its_own_overlay() {
        let mut join = with_detector();
        assert!(join.add_frame("a", 100 * MS, 101 * MS).is_empty());
        assert!(join.add_frame("b", 116 * MS, 117 * MS).is_empty());
        assert_eq!(join.next_deadline_ns(), Some(151 * MS));
        // The detector answers for a, then b.
        let ready = join.add_overlay(stamped(100 * MS), 105 * MS);
        assert_eq!(captures(&ready), vec![(100 * MS, true)]);
        assert_eq!(ready[0].frame, "a");
        let ready = join.add_overlay(stamped(116 * MS), 121 * MS);
        assert_eq!(captures(&ready), vec![(116 * MS, true)]);
        assert_eq!(join.drawn, 2);
        assert_eq!(join.next_deadline_ns(), None);
    }

    #[test]
    fn a_frame_whose_overlay_never_comes_goes_out_at_its_deadline() {
        let mut join = with_detector();
        join.add_frame("a", 100 * MS, 101 * MS);
        assert!(join.expire(150 * MS).is_empty());
        let ready = join.expire(151 * MS);
        assert_eq!(captures(&ready), vec![(100 * MS, false)]);
        assert_eq!(join.missed, 1);
        // They turn up after all: too late, and not drawn on anything else.
        let unmatched = join.unmatched;
        assert!(join.add_overlay(stamped(100 * MS), 160 * MS).is_empty());
        assert_eq!(join.unmatched, unmatched + 1);
    }

    #[test]
    fn later_overlays_release_earlier_frames_that_missed_theirs() {
        let mut join = with_detector();
        join.add_frame("a", 100 * MS, 101 * MS);
        join.add_frame("b", 116 * MS, 117 * MS);
        // The detector skipped a.
        let ready = join.add_overlay(stamped(116 * MS), 120 * MS);
        assert_eq!(captures(&ready), vec![(100 * MS, false), (116 * MS, true)]);
    }

    #[test]
    fn overlays_for_skipped_frames_are_ignored() {
        let mut join = with_detector();
        // The stream took frames at 100 and 133 ms, skipping 116 to keep to 30 fps.
        join.add_frame("a", 100 * MS, 101 * MS);
        assert_eq!(
            captures(&join.add_overlay(stamped(100 * MS), 104 * MS)),
            vec![(100 * MS, true)]
        );
        assert!(join.add_overlay(stamped(116 * MS), 120 * MS).is_empty());
        join.add_frame("c", 133 * MS, 134 * MS);
        assert_eq!(
            captures(&join.add_overlay(stamped(133 * MS), 137 * MS)),
            vec![(133 * MS, true)]
        );
    }

    #[test]
    fn a_silent_detector_stops_holding_up_the_video() {
        let mut join = with_detector();
        join.add_frame("a", 100 * MS, 101 * MS);
        // A second later the detector has died: the waiting frame and the new one go out now.
        let ready = join.add_frame("b", 1100 * MS, 1101 * MS);
        assert_eq!(
            captures(&ready),
            vec![(100 * MS, false), (1100 * MS, false)]
        );
        // It comes back: frames wait again.
        join.add_overlay(stamped(1150 * MS), 1155 * MS);
        assert!(join.add_frame("c", 1200 * MS, 1201 * MS).is_empty());
    }
}
