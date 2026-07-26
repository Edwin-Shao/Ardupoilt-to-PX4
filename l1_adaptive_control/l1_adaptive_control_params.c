/**
 * Automatically start the L1 failure-mode controller
 *
 * The controller remains idle outside the L1 position and altitude
 * failure modes.
 *
 * @boolean
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_INT32(L1_FAIL_EN, 1);
