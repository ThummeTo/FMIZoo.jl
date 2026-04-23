#ifndef config_h
#define config_h

// define class name and unique id
// #define MODEL_IDENTIFIER BouncingBall
#define INSTANTIATION_TOKEN "{1AE5E10D-9521-4DE3-80B9-D0EAAA7D5AF1}"

#define CO_SIMULATION
#define MODEL_EXCHANGE

#define HAS_CONTINUOUS_STATES
#define HAS_EVENT_INDICATORS

#define SET_FLOAT64
#define GET_OUTPUT_DERIVATIVE
#define EVENT_UPDATE

#define FIXED_SOLVER_STEP 1e-3
#define DEFAULT_STOP_TIME 3

#ifndef GET_PARTIAL_DERIVATIVE
#define GET_PARTIAL_DERIVATIVE
#endif

#define GET_INT32
#define SET_INT32

#ifndef NUM_BALLS
#define NUM_BALLS 1
#endif

// Global Variables
typedef enum {
    vr_time = 0, 
    vr_g = 1,  
    vr_v_min = 2,  
    vr_ground_flexibility = 3, 
    vr_base_event_period = 4, 
    vr_nextEventTime = 5
} ValueReferenceGlobal;

#define START_VR 100
#define VARS_PER_BALL 12

#define OFF_H 0
#define OFF_DER_H 1
#define OFF_V 2
#define OFF_DER_V 3
#define OFF_E 4
#define OFF_K 5
#define OFF_DRAG 6
#define OFF_BOUNCE 7
#define OFF_PRE_H 8
#define OFF_PRE_V 9
#define OFF_ERR_H 10
#define OFF_EVENT_G 11

typedef uint32_t ValueReference;

typedef struct {

    double h;
    double der_h;
    double v;
    double der_v;

    double e;
    double k;
    double drag_coefficient;
    
    int32_t bounce_count;
    double pre_h;
    double pre_v;
    double error_h_start;
    double event_indicator_ground;
  
} BallData;

typedef struct {
    BallData balls[NUM_BALLS];

    double g;
    double v_min;
    double ground_flexibility;
    double base_event_period;
    double current_event_period;
    double nextEventTime;

    double event_kick_velocity;

} ModelData;

#endif /* config_h */
