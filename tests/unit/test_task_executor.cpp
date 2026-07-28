#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "rally/core/task_executor.hpp"
#include "rally/core/clock.hpp"

using namespace rally::core;

class MockControlTask : public ITask {
public:
    int execution_count = 0;
    
    void execute(uint64_t /*current_time_us*/) override {
        execution_count++;
    }
    
    const char* get_name() const override { return "MockControl_500Hz"; }
};

TEST_CASE("TaskExecutor respects period configuration", "[scheduler]") {
    Clock::init(ClockMode::SIMULATED);
    
    TaskExecutor executor;
    MockControlTask control_task;
    
    // 500Hz = 2000us period, with a 1000us deadline
    REQUIRE(executor.register_task(&control_task, 500, 1000) == true);
    REQUIRE(control_task.execution_count == 0);
    
    // Advance to 1000us (not enough for one period)
    Clock::update_sim_time(1000);
    executor.step();
    REQUIRE(control_task.execution_count == 0);

    // Advance to 2000us (exactly one period)
    Clock::update_sim_time(2000);
    executor.step();
    REQUIRE(control_task.execution_count == 1);

    // Advance to 4000us (exactly two periods total)
    Clock::update_sim_time(4000);
    executor.step();
    REQUIRE(control_task.execution_count == 2);
}

TEST_CASE("TaskExecutor anti-windup prevents catch-up storms", "[scheduler]") {
    Clock::init(ClockMode::SIMULATED);
    
    TaskExecutor executor;
    MockControlTask control_task;
    
    executor.register_task(&control_task, 500, 1000); // 2000us period
    
    // Simulate a massive block in the system (e.g. OS paused the process)
    // Time jumps to 10,000us. It missed 5 periods. 
    Clock::update_sim_time(10000);
    
    // It should execute ONCE to service the current slot...
    executor.step();
    REQUIRE(control_task.execution_count == 1);
    
    // ...and then snap its internal tracker forward. 
    // Stepping again immediately should NOT fire off 4 more delayed executions.
    executor.step();
    REQUIRE(control_task.execution_count == 1);
}