/**
 * @file deployment.h
 * @brief Parachute deployment module
 */

#ifndef DEPLOYMENT_H
#define DEPLOYMENT_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    DEPLOY_IDLE,
    DEPLOY_READY,
    DEPLOY_TRIGGERED,
    DEPLOY_COMPLETE,
    DEPLOY_FAILED
} DeploymentState;

void deployment_init(void);
void deployment_arm(void);
void deployment_disarm(void);
bool deployment_trigger(float altitude_m);
bool deployment_is_armed(void);
DeploymentState deployment_get_state(void);
bool deployment_is_deployed(void);
bool deployment_test(void);
void deployment_reset(void);

#endif // DEPLOYMENT_H
