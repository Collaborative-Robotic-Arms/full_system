#pragma once

#include <string>
#include <memory>
#include <atomic>

namespace dual_arms_mtc {

/**
 * @brief Defines the type of operation being executed
 * 
 * HANDOVER: Sequential control required - one arm waits for the other
 * PICK_PLACE: Parallel control - both arms can operate independently
 * IDLE: No active operation
 */
enum class OperationType {
    IDLE,
    HANDOVER,       // Sequential: AR4 picks → intermediate → ABB picks
    PICK_PLACE,     // Parallel: Both arms pick/place independently
    SYNCHRONIZED    // Synchronized movement of both arms without handover
};

/**
 * @brief Defines which execution model to use during operations
 * 
 * SEQUENTIAL: Operations must wait for each other (used in handover)
 * PARALLEL: Operations can execute simultaneously (used in pick/place)
 */
enum class ExecutionModel {
    SEQUENTIAL,
    PARALLEL
};

/**
 * @brief Defines the phase of operation within a sequence
 */
enum class OperationPhase {
    IDLE,
    INITIATING,         // Starting phase
    ARM1_ACTIVE,        // First arm executing (AR4)
    ARM1_COMPLETE,      // First arm completed its task
    ARM2_ACTIVE,        // Second arm executing in response (ABB)
    ARM2_COMPLETE,      // Second arm completed
    COMPLETION,         // Finalizing operation
    ERROR
};

/**
 * @brief Operation context encapsulating the current operation state
 */
struct OperationContext {
    OperationType type = OperationType::IDLE;
    ExecutionModel model = ExecutionModel::PARALLEL;
    OperationPhase phase = OperationPhase::IDLE;
    
    // Tracking
    std::string operation_id;
    uint64_t start_timestamp_ms = 0;
    uint64_t phase_timestamp_ms = 0;
    
    // For sequential operations
    bool arm1_ready = false;
    bool arm1_complete = false;
    bool arm2_ready = false;
    bool arm2_complete = false;
    
    // Timeout tracking
    uint64_t max_phase_duration_ms = 10000;  // 10 seconds per phase
    
    std::string to_string() const {
        std::string type_str = (type == OperationType::HANDOVER) ? "HANDOVER" :
                              (type == OperationType::PICK_PLACE) ? "PICK_PLACE" :
                              (type == OperationType::SYNCHRONIZED) ? "SYNCHRONIZED" : "IDLE";
        
        std::string model_str = (model == ExecutionModel::SEQUENTIAL) ? "SEQUENTIAL" : "PARALLEL";
        
        std::string phase_str = (phase == OperationPhase::IDLE) ? "IDLE" :
                               (phase == OperationPhase::INITIATING) ? "INITIATING" :
                               (phase == OperationPhase::ARM1_ACTIVE) ? "ARM1_ACTIVE" :
                               (phase == OperationPhase::ARM1_COMPLETE) ? "ARM1_COMPLETE" :
                               (phase == OperationPhase::ARM2_ACTIVE) ? "ARM2_ACTIVE" :
                               (phase == OperationPhase::ARM2_COMPLETE) ? "ARM2_COMPLETE" :
                               (phase == OperationPhase::COMPLETION) ? "COMPLETION" : "ERROR";
        
        return "OperationContext[type=" + type_str + ", model=" + model_str + 
               ", phase=" + phase_str + ", id=" + operation_id + "]";
    }
};

}  // namespace dual_arms_mtc
