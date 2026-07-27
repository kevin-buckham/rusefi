/*
 * @file test_error_cookie.cpp
 *
 * Every deliberately-written ErrorCookie must survive errorHandlerInit()'s
 * boot-time sanitize (which erases unknown values as first-power-on garbage).
 * Guards against the #9931 regression where NmiReset/SoftReset/OpenBltJump
 * were silently reset to None by a stale three-value whitelist.
 */

#include "pch.h"

TEST(errorCookie, writtenCookiesSurviveBootSanitize) {
	// crash paths
	EXPECT_TRUE(isKnownErrorCookie(ErrorCookie::FirmwareError));
	EXPECT_TRUE(isKnownErrorCookie(ErrorCookie::HardFault));
	EXPECT_TRUE(isKnownErrorCookie(ErrorCookie::ChibiOsPanic));
	// deliberate/exceptional reset paths (#9931)
	EXPECT_TRUE(isKnownErrorCookie(ErrorCookie::NmiReset));
	EXPECT_TRUE(isKnownErrorCookie(ErrorCookie::SoftReset));
	EXPECT_TRUE(isKnownErrorCookie(ErrorCookie::OpenBltJump));
}

TEST(errorCookie, garbageAndNoneAreSanitized) {
	EXPECT_FALSE(isKnownErrorCookie(ErrorCookie::None));
	// random first-power-on backup-SRAM content
	EXPECT_FALSE(isKnownErrorCookie((ErrorCookie)0x12345678));
	EXPECT_FALSE(isKnownErrorCookie((ErrorCookie)0xffffffff));
}
