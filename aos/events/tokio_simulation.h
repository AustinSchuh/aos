#ifndef AOS_EVENTS_TOKIO_SIMULATION_H_
#define AOS_EVENTS_TOKIO_SIMULATION_H_

#include "aos/events/event_loop_c.h"
#include "aos/events/simulated_event_loop.h"

// A simulation with tokio as the outer loop, for a replay or a test assembled
// in C++ with Rust applications in it.  See
// documentation/adr/0008-tokio-on-the-event-loop-thread.md.
//
// C++ builds the simulation as usual (a SimulatedEventLoopFactory, a LogReader
// registered with it, its C++ applications), makes an event loop for each Rust
// application, and hands each to the application's own function for adding it
// to an aos_tokio_simulation_t.  Then TokioSimulation::Run() runs the
// simulation in place of the factory's Run(): tokio drives the scheduler's Aio,
// and runs the Rust applications' tasks between simulated events.
//
// Implemented in Rust, in aos/events/tokio_event_loop.rs.

extern "C" {

typedef struct aos_tokio_simulation_t aos_tokio_simulation_t;

aos_tokio_simulation_t *aos_tokio_simulation_create(
    aos_simulated_event_loop_factory_t *factory);
// Runs the simulation until it stops, and destroys it.
void aos_tokio_simulation_run(aos_tokio_simulation_t *simulation);
// Destroys a simulation that hasn't run.
void aos_tokio_simulation_destroy(aos_tokio_simulation_t *simulation);
}

namespace aos {

class TokioSimulation {
 public:
  // factory must outlive this, and not run otherwise.
  explicit TokioSimulation(SimulatedEventLoopFactory *factory)
      : simulation_(aos_tokio_simulation_create(
            reinterpret_cast<aos_simulated_event_loop_factory_t *>(factory))) {}
  TokioSimulation(const TokioSimulation &) = delete;
  TokioSimulation &operator=(const TokioSimulation &) = delete;
  ~TokioSimulation() {
    if (simulation_ != nullptr) {
      aos_tokio_simulation_destroy(simulation_);
    }
  }

  // What Rust applications' functions take to add themselves.
  aos_tokio_simulation_t *get() { return simulation_; }

  // An event loop as Rust applications take it.
  static aos_event_loop_t *ForRust(EventLoop *event_loop) {
    return reinterpret_cast<aos_event_loop_t *>(event_loop);
  }

  // Runs the simulation until it stops, in place of the factory's Run().
  void Run() {
    aos_tokio_simulation_run(simulation_);
    simulation_ = nullptr;
  }

 private:
  aos_tokio_simulation_t *simulation_;
};

}  // namespace aos

#endif  // AOS_EVENTS_TOKIO_SIMULATION_H_
