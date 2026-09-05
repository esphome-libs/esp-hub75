// SPDX-FileCopyrightText: 2026 Stuart Parmenter
// SPDX-License-Identifier: MIT
// Execute production lifecycle code with controlled DMA completions. These
// mocks test ownership and failure handling, not peripheral timing or throughput.
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <future>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include "host/esp_heap_caps.h"
// Observe ownership at frame boundaries without adding a production test API.
#define private public
#include "../../components/hub75/src/platforms/parlio_stream/parlio_stream_dma.h"
#undef private

using namespace std::chrono_literals;
struct HostTask {
  std::thread thread;
  uint32_t notifications{0};
  bool suspended{false};
  bool deleted{false};
};
struct HostSemaphore {
  unsigned count;
  bool deleted{false};
};
struct Transaction {
  const void *payload;
  std::vector<uint16_t> snapshot;
};
struct HostParlio {
  parlio_tx_unit_config_t config;
  parlio_tx_event_callbacks_t callbacks;
  void *context{nullptr};
  bool enabled{false};
  std::deque<Transaction> queue;
};

namespace {
std::mutex runtime_mutex;
std::condition_variable runtime_changed;
thread_local HostTask *current_task = nullptr;
thread_local bool inside_isr = false;
std::vector<std::unique_ptr<HostTask>> tasks;
std::vector<std::unique_ptr<HostSemaphore>> semaphores;
struct Allocation {
  size_t bytes;
  unsigned capabilities;
};
std::map<void *, Allocation> allocations;
HostParlio *active_unit = nullptr;
std::vector<const void *> submissions;
size_t submission_attempts = 0;
size_t fail_submission = 0;
bool fail_task_creation = false;
bool fail_enable = false;
bool callback_during_disable = false;
unsigned callback_count = 0;
unsigned next_started_in_isr = 0;
struct TaskExit {};

void check_busy_buffers() {
  if (!active_unit)
    return;
  for (const auto &transaction : active_unit->queue) {
    assert(std::memcmp(transaction.payload, transaction.snapshot.data(), transaction.snapshot.size() * 2) == 0 &&
           "producer modified a buffer still owned by DMA");
  }
}

template<typename Predicate> void await(Predicate predicate) {
  std::unique_lock<std::mutex> lock(runtime_mutex);
  assert(runtime_changed.wait_for(lock, 3s, predicate) && "production worker failed to make progress");
}

void complete_batch(unsigned count) {
  // Model the C6 single-core ISR: callbacks run before descriptor recycling,
  // and the next queued descriptor starts before the worker can resume.
  std::lock_guard<std::mutex> lock(runtime_mutex);
  assert(active_unit && active_unit->enabled && active_unit->queue.size() >= count);
  for (unsigned i = 0; i < count; ++i) {
    check_busy_buffers();
    inside_isr = true;
    active_unit->callbacks.on_trans_done(active_unit, nullptr, active_unit->context);
    inside_isr = false;
    ++callback_count;
    active_unit->queue.pop_front();
    if (!active_unit->queue.empty())
      ++next_started_in_isr;
  }
  runtime_changed.notify_all();
}

void reset_runtime() {
  assert(!active_unit && allocations.empty());
  for (const auto &task : tasks)
    assert(task->deleted && !task->thread.joinable());
  for (const auto &semaphore : semaphores)
    assert(semaphore->deleted);
  tasks.clear();
  semaphores.clear();
  submissions.clear();
  submission_attempts = fail_submission = 0;
  fail_task_creation = fail_enable = callback_during_disable = false;
  callback_count = next_started_in_isr = 0;
}

Hub75Config config() {
  Hub75Config result;
  result.panel_width = 64;
  result.panel_height = 32;
  result.double_buffer = true;
  return result;
}
}  // namespace

BaseType_t xTaskCreate(void (*entry)(void *), const char *, uint32_t, void *argument, UBaseType_t,
                       TaskHandle_t *handle) {
  if (fail_task_creation)
    return pdFALSE;
  auto task = std::make_unique<HostTask>();
  *handle = task.get();
  HostTask *raw = task.get();
  tasks.push_back(std::move(task));
  raw->thread = std::thread([=] {
    current_task = raw;
    try {
      entry(argument);
    } catch (const TaskExit &) {
    }
  });
  return pdPASS;
}
void vTaskSuspend(TaskHandle_t handle) {
  assert(handle == nullptr && current_task);
  std::unique_lock<std::mutex> lock(runtime_mutex);
  current_task->suspended = true;
  runtime_changed.notify_all();
  runtime_changed.wait(lock, [] { return current_task->deleted; });
  throw TaskExit{};
}
void vTaskDelete(TaskHandle_t task) {
  assert(task && task != current_task);
  {
    std::lock_guard<std::mutex> lock(runtime_mutex);
    assert(!active_unit && "ISR must be removed before callback task handle is freed");
    task->deleted = true;
    runtime_changed.notify_all();
  }
  task->thread.join();
}
void xTaskNotifyGive(TaskHandle_t task) {
  assert(!inside_isr);
  std::lock_guard<std::mutex> lock(runtime_mutex);
  assert(task && !task->deleted);
  ++task->notifications;
  runtime_changed.notify_all();
}
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken) {
  assert(inside_isr && task && !task->deleted);
  // Caller holds runtime_mutex throughout the simulated ISR.
  ++task->notifications;
  *woken = pdTRUE;
}
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks) {
  assert(current_task && !inside_isr);
  std::unique_lock<std::mutex> lock(runtime_mutex);
  runtime_changed.wait_for(lock, std::chrono::milliseconds(ticks), [] { return current_task->notifications != 0; });
  const uint32_t result = current_task->notifications;
  if (clear)
    current_task->notifications = 0;
  else if (result)
    --current_task->notifications;
  return result;
}
TaskHandle_t xTaskGetCurrentTaskHandle() { return current_task; }
void vTaskDelay(TickType_t ticks) { std::this_thread::sleep_for(std::chrono::milliseconds(ticks)); }
SemaphoreHandle_t xSemaphoreCreateBinary() {
  auto semaphore = std::make_unique<HostSemaphore>();
  semaphore->count = 0;
  auto *result = semaphore.get();
  semaphores.push_back(std::move(semaphore));
  return result;
}
SemaphoreHandle_t xSemaphoreCreateMutex() {
  auto *result = xSemaphoreCreateBinary();
  result->count = 1;
  return result;
}
BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t ticks) {
  std::unique_lock<std::mutex> lock(runtime_mutex);
  assert(semaphore && !semaphore->deleted && !inside_isr);
  const auto available = [&] { return semaphore->count != 0 || semaphore->deleted; };
  if (ticks == portMAX_DELAY)
    runtime_changed.wait(lock, available);
  else
    runtime_changed.wait_for(lock, std::chrono::milliseconds(ticks), available);
  assert(!semaphore->deleted && "semaphore freed while another task was waiting");
  if (!semaphore->count)
    return pdFALSE;
  --semaphore->count;
  return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore) {
  std::lock_guard<std::mutex> lock(runtime_mutex);
  assert(semaphore && !semaphore->deleted && !inside_isr);
  const bool was_full = semaphore->count != 0;
  semaphore->count = 1;
  runtime_changed.notify_all();
  return was_full ? pdFALSE : pdTRUE;
}
void vSemaphoreDelete(SemaphoreHandle_t semaphore) {
  std::lock_guard<std::mutex> lock(runtime_mutex);
  assert(!semaphore->deleted);
  semaphore->deleted = true;
  runtime_changed.notify_all();
}
void *heap_caps_calloc(size_t count, size_t bytes, unsigned capabilities) {
  void *pointer = std::calloc(count, bytes);
  assert(pointer);
  allocations[pointer] = {count * bytes, capabilities};
  return pointer;
}
void *heap_caps_aligned_calloc(size_t, size_t count, size_t bytes, unsigned capabilities) {
  return heap_caps_calloc(count, bytes, capabilities);
}
void heap_caps_free(void *pointer) {
  if (!pointer)
    return;
  assert(!active_unit && "DMA unit must be destroyed before freeing payloads");
  assert(allocations.erase(pointer) == 1);
  std::free(pointer);
}
esp_err_t parlio_new_tx_unit(const parlio_tx_unit_config_t *config, parlio_tx_unit_handle_t *unit) {
  std::lock_guard<std::mutex> lock(runtime_mutex);
  assert(!active_unit);
  assert(config->max_transfer_size < 65536 && config->data_width == 16);
  active_unit = new HostParlio;
  active_unit->config = *config;
  *unit = active_unit;
  return ESP_OK;
}
esp_err_t parlio_tx_unit_register_event_callbacks(parlio_tx_unit_handle_t unit,
                                                  const parlio_tx_event_callbacks_t *callbacks, void *context) {
  unit->callbacks = *callbacks;
  unit->context = context;
  return ESP_OK;
}
esp_err_t parlio_tx_unit_enable(parlio_tx_unit_handle_t unit) {
  std::lock_guard<std::mutex> lock(runtime_mutex);
  assert(unit->queue.size() == unit->config.trans_queue_depth && "prefill and prequeue before first clock");
  if (fail_enable)
    return ESP_FAIL;
  unit->enabled = true;
  runtime_changed.notify_all();
  return ESP_OK;
}
esp_err_t parlio_tx_unit_disable(parlio_tx_unit_handle_t unit) {
  std::lock_guard<std::mutex> lock(runtime_mutex);
  check_busy_buffers();
  if (callback_during_disable && !unit->queue.empty()) {
    inside_isr = true;
    unit->callbacks.on_trans_done(unit, nullptr, unit->context);
    inside_isr = false;
    ++callback_count;
  }
  unit->enabled = false;
  // ESP-IDF leaves pending transactions queued; deleting/recreating the unit
  // before restart is necessary. Deliberately retain them here too.
  runtime_changed.notify_all();
  return ESP_OK;
}
esp_err_t parlio_del_tx_unit(parlio_tx_unit_handle_t unit) {
  std::lock_guard<std::mutex> lock(runtime_mutex);
  assert(unit == active_unit && !unit->enabled);
  check_busy_buffers();
  delete unit;
  active_unit = nullptr;
  runtime_changed.notify_all();
  return ESP_OK;
}
esp_err_t parlio_tx_unit_transmit(parlio_tx_unit_handle_t unit, const void *payload, size_t bits,
                                  const parlio_transmit_config_t *config) {
  assert(!inside_isr && "ESP-IDF transmit is not an ISR API");
  std::lock_guard<std::mutex> lock(runtime_mutex);
  check_busy_buffers();
  ++submission_attempts;
  if (submission_attempts == fail_submission)
    return ESP_FAIL;
  assert(bits != 0 && bits % 32 == 0 && bits / 8 <= unit->config.max_transfer_size);
  assert(config->flags.queue_nonblocking && !config->flags.loop_transmission);
  assert(config->idle_value & hub75::StreamEncoder::OE);
  assert(unit->queue.size() < unit->config.trans_queue_depth);
  for (const auto &transaction : unit->queue)
    assert(transaction.payload != payload && "DMA slot reused before completion");
  if (submissions.empty()) {
    unsigned prefilled = 0;
    for (const auto &[pointer, allocation] : allocations) {
      if (!(allocation.capabilities & MALLOC_CAP_DMA))
        continue;
      const auto *words = static_cast<const uint16_t *>(pointer);
      const bool initialized =
          std::any_of(words, words + allocation.bytes / 2, [](uint16_t word) { return word != 0; });
      assert(initialized && "all staging slots must be prepared before any submission");
      ++prefilled;
    }
    assert(prefilled == unit->config.trans_queue_depth);
  }
  const auto *words = static_cast<const uint16_t *>(payload);
  unit->queue.push_back({payload, {words, words + bits / 16}});
  submissions.push_back(payload);
  runtime_changed.notify_all();
  return ESP_OK;
}
esp_err_t parlio_tx_unit_wait_all_done(parlio_tx_unit_handle_t, int) {
  assert(false && "continuous streaming cannot wait for the entire queue to become empty");
  return ESP_FAIL;
}

int main() {
  // Multiple completions in one wake must reclaim all slots in FIFO order.
  {
    auto settings = config();
    hub75::ParlioStreamDma dma(settings);
    assert(dma.init());
    await([] { return submissions.size() == 3; });
    for (unsigned count : {2U, 1U, 3U, 2U}) {
      const size_t before = submissions.size();
      complete_batch(count);
      await([&] { return submissions.size() == before + count; });
    }
    for (size_t i = 3; i < submissions.size(); ++i)
      assert(submissions[i] == submissions[i % 3]);
    assert(next_started_in_isr > 0);
    callback_during_disable = true;
    dma.stop_transfer();
    assert(!active_unit && callback_count == 9);
    dma.start_transfer();
    assert(dma.running_);
    dma.shutdown();
  }
  reset_runtime();
  // Every partial startup failure must release the worker/unit/payloads.
  for (size_t failure : {1U, 2U, 3U}) {
    fail_submission = failure;
    auto settings = config();
    hub75::ParlioStreamDma dma(settings);
    assert(!dma.init());
    dma.shutdown();
    reset_runtime();
  }
  for (bool task_failure : {false, true}) {
    fail_task_creation = task_failure;
    fail_enable = !task_failure;
    auto settings = config();
    hub75::ParlioStreamDma dma(settings);
    assert(!dma.init());
    dma.shutdown();
    reset_runtime();
  }
  // A waiting flip is released on stop and on a submission failure.
  for (bool submission_failure : {false, true}) {
    auto settings = config();
    hub75::ParlioStreamDma dma(settings);
    assert(dma.init());
    auto flip = std::async(std::launch::async, [&] { dma.flip_buffer(); });
    for (;;) {
      xSemaphoreTake(dma.state_mutex_, portMAX_DELAY);
      const bool pending = dma.flip_pending_;
      xSemaphoreGive(dma.state_mutex_);
      if (pending)
        break;
      std::this_thread::yield();
    }
    if (submission_failure) {
      fail_submission = submission_attempts + 1;
      complete_batch(1);
      assert(flip.wait_for(3s) == std::future_status::ready);
      dma.stop_transfer();
    } else {
      // Destruction must also wait for the flip waiter before deleting semaphores.
      dma.shutdown();
      assert(flip.wait_for(3s) == std::future_status::ready);
    }
    flip.get();
    dma.shutdown();
    reset_runtime();
  }
  // The compact front buffer stays immutable until the last chunk has copied
  // it. A flip may release its storage while old DMA staging copies remain.
  {
    auto settings = config();
    hub75::ParlioStreamDma dma(settings);
    assert(dma.init());
    dma.fill(0, 0, 64, 32, 255, 0, 0);
    auto flip = std::async(std::launch::async, [&] { dma.flip_buffer(); });
    for (;;) {
      xSemaphoreTake(dma.state_mutex_, portMAX_DELAY);
      const bool pending = dma.flip_pending_;
      xSemaphoreGive(dma.state_mutex_);
      if (pending)
        break;
      std::this_thread::yield();
    }
    auto draw_next = std::async(std::launch::async, [&] { dma.fill(0, 0, 64, 32, 0, 255, 0); });
    assert(draw_next.wait_for(5ms) == std::future_status::timeout);
    for (unsigned attempt = 0; flip.wait_for(0ms) != std::future_status::ready; ++attempt) {
      assert(attempt < 200);
      complete_batch(1);
      await([] { return active_unit->queue.size() == 3; });
    }
    flip.get();
    draw_next.get();
    xSemaphoreTake(dma.state_mutex_, portMAX_DELAY);
    assert(dma.encoder_.pixels_ == dma.pixels_[dma.front_]);
    const auto *front = static_cast<const uint8_t *>(dma.pixels_[dma.front_]);
    for (size_t i = 0; i < dma.encoder_.storage_bytes(); i += 3) {
      assert(front[i] == 255 && front[i + 1] == 0 && front[i + 2] == 0);
    }
    xSemaphoreGive(dma.state_mutex_);
    complete_batch(2);
    await([] { return active_unit->queue.size() == 3; });
    dma.shutdown();
  }
  reset_runtime();
  std::cout << "C6 production scheduler: ownership, batched ISR completions, startup/failure, stop/restart, and flip "
               "lifetime passed\n";
}
