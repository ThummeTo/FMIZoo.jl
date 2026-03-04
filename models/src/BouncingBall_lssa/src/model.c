#include <math.h>  // for fabs()
#include <float.h> // for DBL_MIN
#include "model.h"


#ifndef NUM_BALLS
#define NUM_BALLS 1
#endif

#define START_VR 100
#define VARS_PER_BALL 12

#define V_MIN (0.1)
#define EVENT_EPSILON (1e-10)

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

#define VR_TIME 0
#define VR_G 1
#define VR_V_MIN 2
#define VR_GROUND_FLEX 3
#define VR_BASE_PERIOD 4
#define VR_NEXT_EVENT 5

static inline double g_eff(ModelInstance* comp, BallData* b)
{
    return M(g) + (b->k * comp->time);
}

void setStartValues(ModelInstance *comp) {
    M(g) = -9.81;
    M(v_min) = V_MIN;
    M(ground_flexibility) = 0.1;
    M(base_event_period) = 1.0;
    M(current_event_period) = M(base_event_period) * 0.7;
    M(event_kick_velocity) = 2.0;
    M(nextEventTime) = M(base_event_period);

    for (int i = 0; i < NUM_BALLS; i++) {
        BallData* b = &comp->modelData.balls[i];

        b->h = 10.0 + (double) i;
        b->v = 0.0;
        b->e = 0.7;
        b->k = 0.0;
        b->drag_coefficient = 0.1;
        b->bounce_count = 0;

        b->pre_h = b->h;
        b->pre_v = b->v;
        b->der_h = 0;
        b->der_v = 0;
    }
}

Status calculateValues(ModelInstance *comp) {
    UNUSED(comp);
    // nothing to do
    return OK;
}

Status getFloat64(ModelInstance* comp, ValueReference vr, double values[], size_t nValues, size_t* index) {

    ASSERT_NVALUES(1);

    if (vr < START_VR) {
        switch (vr) {
            case vr_time:
                values[(*index)++] = comp->time;
                return OK;
            case vr_g:
                values[(*index)++] = M(g);
                return OK;
            case vr_v_min:
                values[(*index)++] = V_MIN;
                return OK;
            case vr_ground_flexibility:
                values[(*index)++] = M(ground_flexibility);
                return OK;
            case vr_base_event_period:
                values[(*index)++] = M(base_event_period);
                return OK;
            case vr_nextEventTime:
                values[(*index)++] = comp->nextEventTimeDefined ? comp->nextEventTime : INFINITY;
                return OK;  
            default:
                logError(comp, "Global GetFloat64 unknown VR %u.", vr);
                return Error;
        }
    }

    size_t offset = vr - START_VR;
    size_t ballIdx = offset / VARS_PER_BALL;
    size_t varType = offset % VARS_PER_BALL;

    if (ballIdx >= NUM_BALLS) return Error;

    BallData* b = &comp->modelData.balls[ballIdx];

    switch (varType) {
        case OFF_H:
            values[(*index)++] = b->h;
            return OK;
        case OFF_DER_H:
        case OFF_V:
            values[(*index)++] = b->v;
            return OK;
        case OFF_DER_V:
        case OFF_E:
            values[(*index)++] = b->e;
            return OK;
        case OFF_K:
            values[(*index)++] = b->k;
            return OK;
        case OFF_DRAG:
            values[(*index)++] = b->drag_coefficient;
            return OK;
        case OFF_PRE_H:
            values[(*index)++] = b->pre_h;
            return OK;
        case OFF_PRE_V:
            values[(*index)++] = b->pre_v;
            return OK;
        case OFF_ERR_H:
            values[(*index)++] = -(b->h);
            return OK;
        case OFF_EVENT_G:
            values[(*index)++] = b->h - (-(M(ground_flexibility) * (1.0 - b->e)));
            return OK;
        default:
            logError(comp, "Get Float64 is not allowed for value reference %u.", vr);
            return Error;
    }
}


Status setFloat64(ModelInstance* comp, ValueReference vr, double value) {

    if (vr < START_VR) {
        switch (vr) {
            case vr_g:
#if FMI_VERSION > 1
                if (comp->type == ModelExchange &&
                    comp->state != Instantiated &&
                    comp->state != InitializationMode) {
                    logError(comp, "Variable g can only be set after instantiation or in initialization mode.");
                    return Error;
                }
#endif
                M(g) = value; 
                return OK;

            case vr_ground_flexibility:
                M(ground_flexibility) = value;
                return OK;

            case vr_base_event_period:
                M(base_event_period) = value;
                return OK;
                
            case vr_v_min:
                 M(v_min) = value;
                 return OK;

            default: 
                logError(comp, "Cannot set global VR %u", vr);
                return Error;
        }
    }

    size_t offset = vr - START_VR;
    size_t ballIdx = offset / VARS_PER_BALL;
    size_t varType = offset % VARS_PER_BALL;

    if (ballIdx >= NUM_BALLS) return Error;
    
    BallData* b = &comp->modelData.balls[ballIdx];

    switch (varType) {
        case OFF_H:
            b->h = value;
            return OK;

        case OFF_V:
            b->v = value;
            return OK;

        case OFF_E:
            b->e = value;
            return OK;

        case OFF_K:
            b->k = value;
            return OK;

        case OFF_DRAG:
            b->drag_coefficient = value;
            return OK;

        default:
            logError(comp, "Unexpected value reference: %u.", vr);
            return Error;
    }
}

Status getOutputDerivative(ModelInstance *comp, ValueReference valueReference, int order, double *value) {

    if (order != 1) {
        logError(comp, "The output derivative order %d for value reference %u is not available.", order, valueReference);
        return Error;
    }

    if (valueReference < START_VR) return Error; // Globale haben keine Ableitung

    size_t offset = valueReference - START_VR;
    size_t ballIdx = offset / VARS_PER_BALL;
    size_t varType = offset % VARS_PER_BALL;

    if (ballIdx >= NUM_BALLS) return Error;
    BallData* b = &comp->modelData.balls[ballIdx];

    switch (varType) {
    case OFF_H:
        *value = b->v;
        return OK;
    case OFF_V:
        *value = g_eff(comp, b);
        return OK;
    default:
        logError(comp, "The output derivative for value reference %u is not available.", valueReference);
        return Error;
    }
}

Status getPartialDerivative(ModelInstance *comp, ValueReference unknown, ValueReference known, double *pd)
{
    bool isUnkGlobal = (unknown < START_VR);
    size_t idxUnk = 0; 
    size_t typeUnk = 0;

    if (!isUnkGlobal) {
        size_t offset = unknown - START_VR;
        idxUnk = offset / VARS_PER_BALL;
        typeUnk = offset % VARS_PER_BALL;
        if (idxUnk >= NUM_BALLS) return Error;
    }

    bool isKnownGlobal = (known < START_VR);
    size_t idxKnown = 0;
    size_t typeKnown = 0;

    if (!isKnownGlobal) {
        size_t offset = known - START_VR;
        idxKnown = offset / VARS_PER_BALL;
        typeKnown = offset % VARS_PER_BALL;
        if (idxKnown >= NUM_BALLS) return Error;
    }

    if (!isUnkGlobal && !isKnownGlobal && (idxUnk != idxKnown)) {
        *pd = 0.0;
        return OK;
    }

    BallData* b = NULL;
    if (!isUnkGlobal) {
        b = &comp->modelData.balls[idxUnk];
    }

    if (known == vr_time)
    {
        // Global Unknowns
        if (isUnkGlobal) {
            switch (unknown) {
                case vr_g:
                    *pd = 0.0;
                    return OK;
                case vr_nextEventTime:
                    *pd = 1.0;
                    return OK;
            default:
                logError(comp, "Unknown valueReference %u for time derivative.", unknown);
                return Error;
            }

            // Local Unknown
            switch (typeUnk)
            {
                case OFF_H:
                    *pd = b->v;
                    return OK;
                case OFF_V:
                case OFF_DER_H:
                {
                    double effective_drag = b->drag_coefficient * (1.0 - b->e);
                    double drag_accel = effective_drag * b->v * fabs(b->v);
                    *pd = (comp->modelData.g + b->k * comp->time) - drag_accel;
                    return OK;
                }
                case OFF_DER_V:
                    *pd = b->k;
                    return OK;
                case OFF_K:
                    *pd = 0.0;
                    return OK;
                default:
                    logError(comp, "Unknown valueReference %u for time derivative.", unknown);
                    return Error;
            }
        }
    }

    if (!isUnkGlobal)
    {
        switch (typeUnk)
        {
            case OFF_H:
                if (!isKnownGlobal && typeKnown == OFF_H) {
                     *pd = 1.0; return OK; 
                }
                *pd = 0.0; 
                return OK;

            case OFF_DER_H:
                 if (!isKnownGlobal && typeKnown == OFF_V) {
                     *pd = 1.0; return OK; 
                 }
                 *pd = 0.0;
                 return OK;

            case OFF_DER_V:
                if (isKnownGlobal) {
                    if (known == vr_g) { 
                        *pd = 1.0; return OK; 
                    }
                    return OK;
                } 
                else {
                    if (typeKnown == OFF_K) { 
                        *pd = comp->time; 
                        return OK; 
                    }
                    
                    if (typeKnown == OFF_V) {
                        double c = b->drag_coefficient * (1.0 - b->e);
                        *pd = - (c * 2.0 * fabs(b->v));
                        return OK;
                    }
                    
                    if (typeKnown == OFF_E) {
                        double term = b->drag_coefficient * b->v * fabs(b->v);
                        *pd = term;
                        return OK;
                    }
                    
                    if (typeKnown == OFF_DRAG) {
                         double term = (1.0 - b->e) * b->v * fabs(b->v);
                         *pd = -term;
                         return OK;
                    }
                }
                *pd = 0.0;
                return OK;

            case OFF_V:
                 if (!isKnownGlobal && typeKnown == OFF_V) {
                     *pd = 1.0; return OK;
                 }
                 *pd = 0.0;
                 return OK;

            case OFF_EVENT_G:
                if (!isKnownGlobal && typeKnown == OFF_H) {
                    *pd = 1.0; return OK; 
                }
                if (!isKnownGlobal && typeKnown == OFF_E) {
                    *pd = -M(ground_flexibility); 
                    return OK;
                }
                if (isKnownGlobal && known == vr_ground_flexibility) {
                    *pd = 1.0 - b->e;
                    return OK;
                }
                *pd = 0.0;
                return OK;
                
            case OFF_ERR_H:
                if (!isKnownGlobal && typeKnown == OFF_H) {
                    *pd = -1.0; return OK;
                }
                *pd = 0.0;
                return OK;

            default:
                *pd = 0.0;
                return OK;
        }
    }

    if (isUnkGlobal)
    {
        if (unknown == vr_nextEventTime) {
             if (known == vr_base_event_period) {
                 *pd = 1.0; 
                 return OK;
             }
        }
    }

    logError(comp, "Partial derivative of %u w.r.t. %u is not supported.", unknown, known);
    return Error;
}

Status eventUpdate(ModelInstance *comp) {
    comp->valuesOfContinuousStatesChanged = false;
    comp->nominalsOfContinuousStatesChanged = false;
    comp->terminateSimulation = false;

    if (comp->nextEventTimeDefined && comp->time >= comp->nextEventTime)
    {
        for (int i = 0; i < NUM_BALLS; i++) {
            BallData* b = &comp->modelData.balls[i];
            b->pre_v = b->v;
            b->v += M(event_kick_velocity);
        }

        comp->nextEventTime += M(current_event_period);
        comp->valuesOfContinuousStatesChanged = true;
    }

    for (int i = 0; i < NUM_BALLS; i++) {
        BallData* b = &comp->modelData.balls[i];

        double ground_level = -M(ground_flexibility) * (1.0 - b->e);

        if (b->h <= ground_level && b->v < 0) {
            b->bounce_count++;

            b->pre_h = b->h;
            b->pre_v = b->v;

            b->h = ground_level + DBL_MIN;

            b->v = -b->v * b->e;

            if (fabs(b->v) < M(v_min)) {
                b->v = 0.0;
            }

            comp->valuesOfContinuousStatesChanged = true;
        }
    }

    return OK;
}

size_t getNumberOfEventIndicators(ModelInstance* comp) {

    UNUSED(comp);

    return NUM_BALLS;
}

size_t getNumberOfContinuousStates(ModelInstance* comp) {

    UNUSED(comp);

    return  NUM_BALLS * 2;
}

Status getContinuousStates(ModelInstance *comp, double x[], size_t nx) {

    UNUSED(nx);

    for (int i = 0; i < NUM_BALLS; i++) {
            BallData* b = &comp->modelData.balls[i];
            x[2 * i]     = b->h;  // Gerade Indizes = Position
            x[2 * i + 1] = b->v;  // Ungerade Indizes = Geschwindigkeit
        }
        return OK;
}

Status setContinuousStates(ModelInstance *comp, const double x[], size_t nx) {

    UNUSED(nx);

for (int i = 0; i < NUM_BALLS; i++) {
        BallData* b = &comp->modelData.balls[i];
        b->h = x[2 * i];      // Position update
        b->v = x[2 * i + 1];  // Geschwindigkeit update
    }
    return OK;
}

Status getDerivatives(ModelInstance *comp, double dx[], size_t nx) {

    UNUSED(nx);

    size_t k = 0;

    for (int i = 0; i < NUM_BALLS; i++) {
        BallData* b = &comp->modelData.balls[i];
        b->der_h = b->v;

        double current_g = g_eff(comp, b);

        double effective_drag = b->drag_coefficient * (1.0 - b->e);
        double drag_accel = effective_drag * b->v * fabs(b->v);

        b->der_v = current_g - drag_accel;

        if (dx) {
            dx[2 * i]     = b->der_h;
            dx[2 * i + 1] = b->der_v;
        }
    }
    return OK;
}

Status getEventIndicators(ModelInstance *comp, double z[], size_t nz) {

    UNUSED(nz);

    for (int i = 0; i < NUM_BALLS; i++) {
        BallData* b = &comp->modelData.balls[i];
        double ground_level = -M(ground_flexibility) * (1.0 - b->e);
        z[i] = b->h - ground_level;
    }

    return OK;
}

Status getInt32(ModelInstance* comp, ValueReference vr,
                int32_t values[], size_t nValues, size_t* index) {
    ASSERT_NVALUES(1);
    
    if (vr < START_VR) return Error;

    size_t offset = vr - START_VR;
    size_t ballIdx = offset / VARS_PER_BALL;
    size_t varType = offset % VARS_PER_BALL;

    if (ballIdx >= NUM_BALLS) return Error;

    if (varType == OFF_BOUNCE) {
        values[(*index)++] = comp->modelData.balls[ballIdx].bounce_count;
        return OK;
    }
    
    // Keine Int32-Variablen vorgesehen → sauber ablehnen.
    // logError(comp, "getInt32 is not supported for valueReference %u.", vr);
    return Error;
}

Status setInt32(ModelInstance* comp, ValueReference vr, const int32_t values[], size_t nValues, size_t* index)
{
    ASSERT_NVALUES(1);
    if (vr < START_VR) return Error;

    size_t offset = vr - START_VR;
    size_t ballIdx = offset / VARS_PER_BALL;
    size_t varType = offset % VARS_PER_BALL;

    if (ballIdx >= NUM_BALLS) return Error;

    if (varType == OFF_BOUNCE) {
         if (comp->state != EventMode) return Error;
         comp->modelData.balls[ballIdx].bounce_count = values[(*index)++];
         return OK;
    }
    return Error;
}