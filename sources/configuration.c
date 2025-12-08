#include <stdint.h>				// Коротние название int.

#include "configuration.h"		// Свой заголовок.

// Инициализация структуры с настройками.
CFG_t CFG = {
	.AfterChangeMinRPM = 1000,
	.AfterChangeMaxRPM = 4700,

	.IdleTPSLimit = 2,
	.MaxSlipRPM = 60,
	.RearGearInitMaxSpeed = 6,
	.PowerDownMaxTPS = 40,

	.MinPressureSLN = 500,
	.IdlePressureSLN = 72,
	.MinPressureSLU = 72,

	.GlockStartValue = 292,
	.GlockWorkValue = 700,
	.GlockMaxTPS = 26,

	.AdaptationStepRatio = 1,

	.G2EnableAdaptTPS = 1,
	.G2AdaptTPSTempMin = 67,
	.G2AdaptTPSTempMax = 72,
	.G2EnableAdaptTemp = 1,
	.G2AdaptTempMaxTPS = 35,

	.G2EnableAdaptReact = 1,
	.G2AdaptReactMinDRPM = 40,
	.G2AdaptReactTempMin = 60,
	.G2AdaptReactTempMax = 72,
	.G2EnableAdaptRctTemp = 1,
	.G2AdaptRctTempMaxTPS = 35,
	.G2ReactStepSize = 60,

	.G3EnableAdaptTPS = 1,
	.G3AdaptTPSTempMin = 63,
	.G3AdaptTPSTempMax = 72,
	.G3EnableAdaptTemp = 1,
	.G3AdaptTempMaxTPS = 35,

	.SpeedImpulsPerKM = 6000,
	.SpeedCalcCoef = 114,
};
