//! What MCP's `wait_for` waits with: a [`Condition`] on a channel, fed each message the channel
//! gets, and ticked between them for conditions on time.
//!
//! By default it waits for the condition to become true: a message where it's true after one
//! where it was false (or the start, if it was false then).  If it's true already when waiting
//! starts, it says so, and waits for it to go false and true again; without `edge`, that ends
//! the wait at once.
//!
//! A message the condition can't be evaluated on (a field it reads isn't set, an index is past
//! the end) counts as false, and the first such reason is kept to report.
//!
//! This holds no fetchers: [`crate::api`] feeds it what its fetchers find.

use crate::condition::{Condition, EvalError, FieldSummary, Input, Message, Summary};

/// A message kept: its bytes and when it was sent.
#[derive(Clone, Debug, PartialEq)]
pub struct Kept {
    pub data: Vec<u8>,
    pub monotonic_ns: i64,
    pub realtime_ns: i64,
    pub queue_index: u32,
}

/// A message the channel just got, as a fetcher has it.
#[derive(Clone, Copy, Debug)]
pub struct Arrived<'a> {
    pub data: &'a [u8],
    pub monotonic_ns: i64,
    pub realtime_ns: i64,
    pub queue_index: u32,
}

impl Arrived<'_> {
    fn keep(&self) -> Kept {
        Kept {
            data: self.data.to_vec(),
            monotonic_ns: self.monotonic_ns,
            realtime_ns: self.realtime_ns,
            queue_index: self.queue_index,
        }
    }
}

fn as_message(kept: &Kept) -> Message<'_> {
    Message {
        data: &kept.data,
        monotonic_ns: kept.monotonic_ns,
    }
}

/// How a wait ended.
#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    /// Whether the condition came true, rather than the wait timing out.
    pub matched: bool,
    /// Whether it was true when waiting started.
    pub true_at_start: bool,
    pub waited_ns: i64,
    /// Messages evaluated, and messages that went by unseen (the wait fell behind).
    pub checked: u64,
    pub missed: u64,
    /// Messages the condition couldn't be evaluated on, and why the first couldn't.
    pub errors: u64,
    pub first_error: Option<String>,
    /// The message it came true on: None if it timed out, or came true between messages
    /// (stale()) before any arrived.
    pub message: Option<Kept>,
    /// The message before that one, or when it timed out, the last one seen.
    pub before: Option<Kept>,
    /// What the fields the condition reads did.
    pub fields: Vec<FieldSummary>,
}

/// A wait in progress.
pub struct Trigger {
    condition: Condition,
    summary: Summary,
    edge: bool,
    started_ns: i64,
    deadline_ns: i64,
    true_at_start: bool,
    /// What the condition was last time it was evaluated.
    last: bool,
    /// The last message seen.
    latest: Option<Kept>,
    checked: u64,
    missed: u64,
    errors: u64,
    first_error: Option<String>,
}

/// What starting a wait did.
pub enum Started {
    /// It's waiting.
    Waiting(Trigger),
    /// It's done already: the condition is true, and it didn't have to become true.
    Done(Report),
}

impl Trigger {
    /// Starts waiting at `now_ns`, for up to `timeout_ns`, with `latest` the channel's latest
    /// message when it starts.
    pub fn start(
        condition: Condition,
        edge: bool,
        now_ns: i64,
        timeout_ns: i64,
        latest: Option<Arrived>,
    ) -> Started {
        let mut summary = Summary::new(&condition);
        if let Some(latest) = &latest {
            summary.add(latest.data);
        }
        let mut trigger = Trigger {
            condition,
            summary,
            edge,
            started_ns: now_ns,
            deadline_ns: now_ns.saturating_add(timeout_ns),
            true_at_start: false,
            last: false,
            latest: latest.map(|latest| latest.keep()),
            checked: 0,
            missed: 0,
            errors: 0,
            first_error: None,
        };
        let input = Input {
            message: trigger.latest.as_ref().map(as_message),
            previous: None,
            now_ns,
            last_ns: trigger.latest.as_ref().map_or(now_ns, |m| m.monotonic_ns),
        };
        // What it was before waiting is no message of the wait's: it isn't counted.
        trigger.true_at_start = trigger.condition.eval(&input).unwrap_or(false);
        trigger.last = trigger.true_at_start;
        if trigger.true_at_start && !edge {
            let message = trigger.latest.clone();
            return Started::Done(trigger.report(true, now_ns, message, None));
        }
        Started::Waiting(trigger)
    }

    pub fn condition(&self) -> &Condition {
        &self.condition
    }

    /// `result`, the condition's, counting it false if it couldn't be evaluated.
    fn counted(&mut self, result: Result<bool, EvalError>) -> bool {
        match result {
            Ok(result) => result,
            Err(error) => {
                self.errors += 1;
                self.first_error.get_or_insert(error.0);
                false
            }
        }
    }

    /// Whether the condition, now `result`, has come true.
    fn came_true(&mut self, result: bool) -> bool {
        let came = result && (!self.edge || !self.last);
        self.last = result;
        came
    }

    /// Takes in a message the channel got, seen at `now_ns`.  Returns how the wait ended if the
    /// condition came true on it.
    pub fn message(&mut self, arrived: Arrived, now_ns: i64) -> Option<Report> {
        if let Some(latest) = &self.latest {
            let gap = arrived.queue_index.wrapping_sub(latest.queue_index);
            // Anything else is the queue's index wrapping or starting over: nothing to count.
            if gap > 1 && gap < u32::MAX / 2 {
                self.missed += (gap - 1) as u64;
            }
        }
        self.checked += 1;
        self.summary.add(arrived.data);
        let input = Input {
            message: Some(Message {
                data: arrived.data,
                monotonic_ns: arrived.monotonic_ns,
            }),
            previous: self.latest.as_ref().map(as_message),
            now_ns,
            last_ns: arrived.monotonic_ns,
        };
        let result = self.condition.eval(&input);
        let result = self.counted(result);
        let before = self.latest.replace(arrived.keep());
        if self.came_true(result) {
            let message = self.latest.clone();
            return Some(self.report(true, now_ns, message, before));
        }
        None
    }

    /// Checks the time at `now_ns`, once the channel's messages so far have been taken in:
    /// whether a condition on time has come true, or the wait is over.
    pub fn tick(&mut self, now_ns: i64) -> Option<Report> {
        if self.condition.timed() {
            let input = Input {
                message: self.latest.as_ref().map(as_message),
                previous: None,
                now_ns,
                last_ns: self
                    .latest
                    .as_ref()
                    .map_or(self.started_ns, |m| m.monotonic_ns.max(self.started_ns)),
            };
            let result = self.condition.eval(&input);
            let result = self.counted(result);
            if self.came_true(result) {
                let message = self.latest.clone();
                return Some(self.report(true, now_ns, message, None));
            }
        }
        if now_ns >= self.deadline_ns {
            let before = self.latest.clone();
            return Some(self.report(false, now_ns, None, before));
        }
        None
    }

    fn report(
        &self,
        matched: bool,
        now_ns: i64,
        message: Option<Kept>,
        before: Option<Kept>,
    ) -> Report {
        Report {
            matched,
            true_at_start: self.true_at_start,
            waited_ns: now_ns - self.started_ns,
            checked: self.checked,
            missed: self.missed,
            errors: self.errors,
            first_error: self.first_error.clone(),
            message,
            before,
            fields: self.summary.fields().to_vec(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::condition::tests::{message, schema_bytes, Fields};
    use flatbuffers_reflection::reflection::root_as_schema;

    const MS: i64 = 1_000_000;

    fn compile(source: &str) -> Condition {
        let bytes = schema_bytes();
        let schema = root_as_schema(&bytes).unwrap();
        Condition::compile(source, &schema).unwrap()
    }

    fn count(count: i32) -> Vec<u8> {
        message(Fields {
            count,
            ..Default::default()
        })
    }

    fn arrived(data: &[u8], index: u32) -> Arrived<'_> {
        Arrived {
            data,
            monotonic_ns: index as i64 * 10 * MS,
            realtime_ns: 1_000 + index as i64,
            queue_index: index,
        }
    }

    fn waiting(started: Started) -> Trigger {
        match started {
            Started::Waiting(trigger) => trigger,
            Started::Done(report) => panic!("Done already: {report:?}"),
        }
    }

    #[test]
    fn waits_for_it_to_become_true() {
        let start = count(1);
        let mut trigger = waiting(Trigger::start(
            compile("count > 2"),
            true,
            0,
            1000 * MS,
            Some(arrived(&start, 1)),
        ));
        let (two, three) = (count(2), count(3));
        assert_eq!(trigger.message(arrived(&two, 2), 20 * MS), None);
        let report = trigger.message(arrived(&three, 3), 30 * MS).unwrap();
        assert!(report.matched);
        assert!(!report.true_at_start);
        assert_eq!(report.waited_ns, 30 * MS);
        assert_eq!((report.checked, report.missed, report.errors), (2, 0, 0));
        assert_eq!(report.message.unwrap().data, three);
        assert_eq!(report.before.unwrap().data, two);
        assert_eq!(report.fields[0].field, "count");
        assert_eq!(report.fields[0].min, Some(1.0));
        assert_eq!(report.fields[0].max, Some(3.0));
    }

    #[test]
    fn true_already() {
        let start = count(5);
        // Without edge, that's that.
        let Started::Done(report) = Trigger::start(
            compile("count > 2"),
            false,
            0,
            1000 * MS,
            Some(arrived(&start, 1)),
        ) else {
            panic!("Should be done");
        };
        assert!(report.matched && report.true_at_start);
        assert_eq!(report.message.unwrap().data, start);

        // With it, it has to go false and come true again.
        let mut trigger = waiting(Trigger::start(
            compile("count > 2"),
            true,
            0,
            1000 * MS,
            Some(arrived(&start, 1)),
        ));
        let (still, low, high) = (count(6), count(1), count(9));
        assert_eq!(trigger.message(arrived(&still, 2), 20 * MS), None);
        assert_eq!(trigger.message(arrived(&low, 3), 30 * MS), None);
        let report = trigger.message(arrived(&high, 4), 40 * MS).unwrap();
        assert!(report.matched && report.true_at_start);
        assert_eq!(report.message.unwrap().data, high);
    }

    #[test]
    fn times_out_saying_what_it_saw() {
        let mut trigger = waiting(Trigger::start(
            compile("count > 100"),
            true,
            0,
            50 * MS,
            None,
        ));
        let (a, b) = (count(4), count(-3));
        assert_eq!(trigger.message(arrived(&a, 1), 10 * MS), None);
        assert_eq!(trigger.message(arrived(&b, 2), 20 * MS), None);
        assert_eq!(trigger.tick(40 * MS), None);
        let report = trigger.tick(50 * MS).unwrap();
        assert!(!report.matched);
        assert_eq!(report.message, None);
        assert_eq!(report.before.unwrap().data, b);
        assert_eq!(report.fields[0].last.as_deref(), Some("-3"));
        assert_eq!(report.fields[0].min, Some(-3.0));
        assert_eq!(report.fields[0].max, Some(4.0));
    }

    #[test]
    fn counts_what_it_missed() {
        let mut trigger = waiting(Trigger::start(
            compile("count > 100"),
            true,
            0,
            50 * MS,
            None,
        ));
        let (a, b) = (count(1), count(2));
        trigger.message(arrived(&a, 7), 0);
        trigger.message(arrived(&b, 11), 0);
        let report = trigger.tick(50 * MS).unwrap();
        assert_eq!((report.checked, report.missed), (2, 3));
    }

    #[test]
    fn changed_compares_with_the_message_before() {
        let start = count(1);
        let mut trigger = waiting(Trigger::start(
            compile("changed(count)"),
            true,
            0,
            1000 * MS,
            Some(arrived(&start, 1)),
        ));
        let (same, different) = (count(1), count(2));
        assert_eq!(trigger.message(arrived(&same, 2), 0), None);
        let report = trigger.message(arrived(&different, 3), 0).unwrap();
        assert_eq!(report.before.unwrap().data, same);
    }

    #[test]
    fn stale_comes_true_between_messages() {
        let start = count(1);
        let mut trigger = waiting(Trigger::start(
            compile("stale(100ms)"),
            true,
            10 * MS,
            1000 * MS,
            Some(arrived(&start, 1)),
        ));
        let next = count(2);
        assert_eq!(trigger.message(arrived(&next, 5), 60 * MS), None);
        assert_eq!(trigger.tick(100 * MS), None);
        // The last message was sent at 50 ms.
        let report = trigger.tick(151 * MS).unwrap();
        assert!(report.matched);
        assert_eq!(report.message.unwrap().data, next);

        // A channel with nothing on it goes stale from when the wait started.
        let mut trigger = waiting(Trigger::start(
            compile("stale(100ms)"),
            true,
            10 * MS,
            1000 * MS,
            None,
        ));
        assert_eq!(trigger.tick(100 * MS), None);
        let report = trigger.tick(111 * MS).unwrap();
        assert!(report.matched);
        assert_eq!(report.message, None);
    }

    #[test]
    fn counts_what_it_cant_evaluate_as_false() {
        let mut trigger = waiting(Trigger::start(
            compile("left.position > 1"),
            true,
            0,
            50 * MS,
            None,
        ));
        let unset = count(1);
        assert_eq!(trigger.message(arrived(&unset, 1), 0), None);
        let set = message(Fields {
            left: Some((2.0, 0.0)),
            ..Default::default()
        });
        let report = trigger.message(arrived(&set, 2), 0).unwrap();
        assert_eq!(report.errors, 1);
        assert_eq!(report.first_error.as_deref(), Some("left isn't set"));
    }
}
