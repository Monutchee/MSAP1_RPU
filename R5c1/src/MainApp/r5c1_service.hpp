#ifndef MSAP1_R5C1_SERVICE_HPP
#define MSAP1_R5C1_SERVICE_HPP

/*
 * R5 core 1 control service.
 *
 * On the KR260 board R5c1 has no LED, so it uses the comm-only ControlService
 * behaviour unchanged: SET_LED is accepted as an ACK no-op and the LED status
 * fields report zero (the base-class defaults). This subclass exists for
 * symmetry with R5c0Service and as a home for any future R5c1-specific
 * message handling (override handle_custom()).
 */

#include "control_service.hpp"

class R5c1Service : public msap1::ControlService {
public:
	using msap1::ControlService::ControlService;
};

#endif /* MSAP1_R5C1_SERVICE_HPP */
