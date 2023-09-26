/*
Copyright (C) 1997-2001 Id Software, Inc.
Copyright (C) 2022 Chasseur de bots

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

*/

#include "transienteffectssystem.h"
#include "../cgame/cg_local.h"
#include "../client/client.h"
#include "../qcommon/links.h"

#include <cstdlib>
#include <cstring>

TransientEffectsSystem::~TransientEffectsSystem() {
	for( EntityEffect *effect = m_entityEffectsHead, *next = nullptr; effect; effect = next ) { next = effect->next;
		unlinkAndFreeEntityEffect( effect );
	}
	for( PolyEffect *effect = m_polyEffectsHead, *next = nullptr; effect; effect = next ) { next = effect->next;
		unlinkAndFreePolyEffect( effect );
	}
	for( LightEffect *effect = m_lightEffectsHead, *next = nullptr; effect; effect = next ) { next = effect->next;
		unlinkAndFreeLightEffect( effect );
	}
	for( DelayedEffect *effect = m_delayedEffectsHead, *next = nullptr; effect; effect = next ) { next = effect->next;
		unlinkAndFreeDelayedEffect( effect );
	}
}

#define asByteColor( r, g, b, a ) {  \
		(uint8_t)( ( r ) * 255.0f ), \
		(uint8_t)( ( g ) * 255.0f ), \
		(uint8_t)( ( b ) * 255.0f ), \
		(uint8_t)( ( a ) * 255.0f )  \
	}
/*
static const byte_vec4_t kFireCoreReplacementPalette[] {
	asByteColor( 1.00f, 0.67f, 0.56f, 0.56f ),
	asByteColor( 1.00f, 0.67f, 0.50f, 0.67f ),
	asByteColor( 1.00f, 0.56f, 0.42f, 0.50f ),
	asByteColor( 1.00f, 0.56f, 0.42f, 0.28f ),
};*/
static const byte_vec4_t kFireCoreReplacementPalette[] {
    asByteColor( 0.80f, 0.536f, 0.448f, 0.448f ),
    asByteColor( 0.80f, 0.536f, 0.40f, 0.536f ),
    asByteColor( 0.80f, 0.448f, 0.336f, 0.40f ),
    asByteColor( 0.80f, 0.448f, 0.336f, 0.224f )
};

static const byte_vec4_t kFireReplacementPalette[] {
	asByteColor( 1.00f, 0.42f, 0.00f, 0.15f ),
	asByteColor( 1.00f, 0.28f, 0.00f, 0.15f ),
	asByteColor( 1.00f, 0.67f, 0.00f, 0.15f ),
	asByteColor( 1.00f, 0.50f, 0.00f, 0.15f ),
	asByteColor( 1.00f, 0.56f, 0.00f, 0.15f ),
};

static const byte_vec4_t kDarkFireDecayPalette[] {
	asByteColor( 0.3f, 0.2f, 0.2f, 0.3f ),
	asByteColor( 0.2f, 0.2f, 0.2f, 0.4f ),
	asByteColor( 0.3f, 0.1f, 0.1f, 0.3f ),
	asByteColor( 0.2f, 0.1f, 0.1f, 0.3f ),
	asByteColor( 0.3f, 0.2f, 0.2f, 0.3f ),
	asByteColor( 0.0f, 0.0f, 0.0f, 0.3f ),
};

static const byte_vec4_t kLightFireDecayPalette[] {
	asByteColor( 0.7f, 0.7f, 0.7f, 0.1f ),
	asByteColor( 0.3f, 0.3f, 0.3f, 0.2f ),
	asByteColor( 0.4f, 0.4f, 0.4f, 0.2f ),
	asByteColor( 0.5f, 0.5f, 0.5f, 0.3f ),
	asByteColor( 0.6f, 0.6f, 0.6f, 0.2f ),
	asByteColor( 0.7f, 0.7f, 0.7f, 0.1f ),
};

static const byte_vec4_t kFadeOutPalette[] {
        asByteColor( 0.1f, 0.0f, 0.0f, 1.0f ),
        asByteColor( 0.06f, 0.02f, 0.0f, 1.0f ),
        asByteColor( 0.f, 0.f, 0.f, 1.0f )
};

struct FireHullLayerParamsHolder {
    SimulatedHullsSystem::ColorChangeTimelineNode darkColorChangeTimeline[5][4] {
		{
			{ /* Layer 0 */ },
			{
                .activateAtLifetimeFraction = 0.35f, .replacementPalette = kFireCoreReplacementPalette,
				.sumOfDropChanceForThisSegment = 0.0f, .sumOfReplacementChanceForThisSegment = 0.1f,
			},
			{
                .activateAtLifetimeFraction = 0.6f, .replacementPalette = kFireReplacementPalette,
				.sumOfDropChanceForThisSegment = 3.5f, .sumOfReplacementChanceForThisSegment = 0.5f,
			},
            {
                .activateAtLifetimeFraction = 0.85f, .replacementPalette = kFadeOutPalette,
                .sumOfDropChanceForThisSegment = 3.5f, .sumOfReplacementChanceForThisSegment = 0.5f,
                }
		},
        {
            { /* Layer 1 */ },
            {
                    .activateAtLifetimeFraction = 0.35f, .replacementPalette = kFireReplacementPalette,
                    .sumOfDropChanceForThisSegment = 0.0f, .sumOfReplacementChanceForThisSegment = 0.2f,
            },
            {
                    .activateAtLifetimeFraction = 0.6f, .replacementPalette = kFireReplacementPalette,
                    .sumOfDropChanceForThisSegment = 2.5f, .sumOfReplacementChanceForThisSegment = 3.5f,
            },
                {
                        .activateAtLifetimeFraction = 0.85f, .replacementPalette = kFadeOutPalette,
                        .sumOfDropChanceForThisSegment = 3.5f, .sumOfReplacementChanceForThisSegment = 0.5f,
                }
        },
        {
            { /* Layer 2 */ },
            {
                    .activateAtLifetimeFraction = 0.3f, .replacementPalette = kFireReplacementPalette,
                    .sumOfDropChanceForThisSegment = 0.0f, .sumOfReplacementChanceForThisSegment = 0.3f,
            },
            {
                    .activateAtLifetimeFraction = 0.6f, .replacementPalette = kFireReplacementPalette,
                    .sumOfDropChanceForThisSegment = 2.0f, .sumOfReplacementChanceForThisSegment = 4.0f,
                    .allowIncreasingOpacity = true
            },
                {
                        .activateAtLifetimeFraction = 0.85f, .replacementPalette = kFadeOutPalette,
                        .sumOfDropChanceForThisSegment = 3.5f, .sumOfReplacementChanceForThisSegment = 0.5f,
                }
        },
        {
            { /* Layer 3 */ },
            {
                    .activateAtLifetimeFraction = 0.3f, .replacementPalette = kFireReplacementPalette,
                    .sumOfDropChanceForThisSegment = 0.0f, .sumOfReplacementChanceForThisSegment = 0.5f,
            },
            {
                    .activateAtLifetimeFraction = 0.55f,
                    .sumOfDropChanceForThisSegment = 2.0f, .sumOfReplacementChanceForThisSegment = 4.5f,
                    .allowIncreasingOpacity = true
            },
                {
                        .activateAtLifetimeFraction = 0.85f, .replacementPalette = kFadeOutPalette,
                        .sumOfDropChanceForThisSegment = 3.5f, .sumOfReplacementChanceForThisSegment = 0.5f,
                }
        },
        {
            { /* Layer 4 */ },
            {
                    .activateAtLifetimeFraction = 0.1f, .replacementPalette = kFireReplacementPalette,
                    .sumOfDropChanceForThisSegment = 0.0f, .sumOfReplacementChanceForThisSegment = 0.5f,
            },
            {
                    .activateAtLifetimeFraction = 0.5f,
                    .sumOfDropChanceForThisSegment = 2.0f, .sumOfReplacementChanceForThisSegment = 4.5f,
                    .allowIncreasingOpacity = true
            },
                {
                        .activateAtLifetimeFraction = 0.85f, .replacementPalette = kFadeOutPalette,
                        .sumOfDropChanceForThisSegment = 3.5f, .sumOfReplacementChanceForThisSegment = 0.5f,
                }
        }
	};

	SimulatedHullsSystem::HullLayerParams darkHullLayerParams[5] {
		{
			.speed = 22.5f, .finalOffset = 8.0f,
			.speedSpikeChance = 0.05f, .minSpeedSpike = 10.0f, .maxSpeedSpike = 15.0f,
			.biasAlongChosenDir = 30.0f,
			.baseInitialColor = { 1.0f, 0.9f, 0.9f, 1.0f },
			.bulgeInitialColor = { 1.0f, 1.0f, 1.0f, 1.0f },
		},
		{
			.speed = 35.0f, .finalOffset = 6.0f,
			.speedSpikeChance = 0.05f, .minSpeedSpike = 7.5f, .maxSpeedSpike = 15.0f,
			.biasAlongChosenDir = 25.0f,
			.baseInitialColor = { 1.0f, 0.9f, 0.9f, 0.7f },
			.bulgeInitialColor = { 1.0f, 1.0f, 1.0f, 0.9f },
		},
		{
			.speed = 45.0f, .finalOffset = 4.0f,
			.speedSpikeChance = 0.05f, .minSpeedSpike = 7.5f, .maxSpeedSpike = 15.0f,
			.biasAlongChosenDir = 20.0f,
			.baseInitialColor = { 1.0f, 0.9f, 0.4f, 0.7f },
			.bulgeInitialColor = { 1.0f, 0.9f, 0.7f, 0.9f },
		},
		{
			.speed = 52.5f, .finalOffset = 2.0f,
			.speedSpikeChance = 0.05f, .minSpeedSpike = 7.5f, .maxSpeedSpike = 15.0f,
			.biasAlongChosenDir = 20.0f,
			.baseInitialColor = { 1.0f, 0.6f, 0.3f, 0.7f },
			.bulgeInitialColor = { 1.0f, 0.9f, 0.7f, 0.7f },
		},
		{
			.speed = 60.0f, .finalOffset = 0.0f,
			.speedSpikeChance = 0.05f, .minSpeedSpike = 7.5f, .maxSpeedSpike = 15.0f,
			.biasAlongChosenDir = 10.0f,
            .baseInitialColor = { 1.0f, 0.5f, 0.2f, 0.7f },
            .bulgeInitialColor = { 1.0f, 0.7f, 0.4f, 0.7f },
		},
	};

	SimulatedHullsSystem::HullLayerParams lightHullLayerParams[5];
	SimulatedHullsSystem::ColorChangeTimelineNode lightColorChangeTimeline[5][3];

	SimulatedHullsSystem::HullLayerParams darkClusterHullLayerParams[2];
	SimulatedHullsSystem::HullLayerParams lightClusterHullLayerParams[2];

	FireHullLayerParamsHolder() noexcept {
		for( size_t layerNum = 0; layerNum < std::size( darkHullLayerParams ); ++layerNum ) {
			// Set the timeline which is not set inline to reduce boilerplate
			darkHullLayerParams[layerNum].colorChangeTimeline = darkColorChangeTimeline[layerNum];

			std::memcpy( lightColorChangeTimeline[layerNum], darkColorChangeTimeline[layerNum],
						 sizeof( darkColorChangeTimeline[layerNum] ) );

			const size_t lastNodeIndex = std::size( lightColorChangeTimeline[layerNum] ) - 1;

			// Raise it for light hulls so they morph to smoke more aggressively
			lightColorChangeTimeline[layerNum][lastNodeIndex].sumOfReplacementChanceForThisSegment *= 1.5f;

			// Set replacement palettes that differ for these layers
			if( layerNum > 2 ) {
				assert( darkColorChangeTimeline[layerNum][lastNodeIndex].replacementPalette.empty() );
				darkColorChangeTimeline[layerNum][lastNodeIndex].replacementPalette  = kDarkFireDecayPalette;
				assert( lightColorChangeTimeline[layerNum][lastNodeIndex].replacementPalette.empty() );
				lightColorChangeTimeline[layerNum][lastNodeIndex].replacementPalette = kLightFireDecayPalette;
			}

			lightHullLayerParams[layerNum] = darkHullLayerParams[layerNum];
			lightHullLayerParams[layerNum].colorChangeTimeline = lightColorChangeTimeline[layerNum];
		}

		assert( std::size( darkClusterHullLayerParams ) == 2 && std::size( lightClusterHullLayerParams ) == 2 );
		darkClusterHullLayerParams[0]  = std::begin( darkHullLayerParams )[0];
		darkClusterHullLayerParams[1]  = std::end( darkHullLayerParams )[-1];
		lightClusterHullLayerParams[0] = std::begin( lightHullLayerParams )[0];
		lightClusterHullLayerParams[1] = std::end( lightHullLayerParams )[-1];

		for( SimulatedHullsSystem::HullLayerParams *layerParam: { &darkClusterHullLayerParams[0],
																  &darkClusterHullLayerParams[1],
																  &lightClusterHullLayerParams[0],
																  &lightClusterHullLayerParams[1] } ) {
			// Account for small lifetime relatively to the primary hull
			layerParam->minSpeedSpike *= 3.0f;
			layerParam->maxSpeedSpike *= 3.0f;
			layerParam->speed         *= 3.0f;
			// Make it more irregular as well
			layerParam->biasAlongChosenDir *= 1.5f;
		}
	}
};

static const FireHullLayerParamsHolder kFireHullParams;

static const byte_vec4_t kSmokeSoftLayerFadeInPalette[] {
	asByteColor( 0.65f, 0.65f, 0.65f, 0.25f ),
	asByteColor( 0.70f, 0.70f, 0.70f, 0.25f ),
	asByteColor( 0.75f, 0.75f, 0.75f, 0.25f ),
	asByteColor( 0.55f, 0.55f, 0.55f, 0.25f ),
	asByteColor( 0.60f, 0.60f, 0.60f, 0.25f ),
};

static const byte_vec4_t kSmokeHardLayerFadeInPalette[] {
	asByteColor( 0.65f, 0.65f, 0.65f, 0.50f ),
	asByteColor( 0.70f, 0.70f, 0.70f, 0.50f ),
	asByteColor( 0.75f, 0.75f, 0.75f, 0.50f ),
	asByteColor( 0.55f, 0.55f, 0.55f, 0.50f ),
	asByteColor( 0.60f, 0.60f, 0.60f, 0.50f ),
};

static const SimulatedHullsSystem::ColorChangeTimelineNode kSmokeHullSoftLayerColorChangeTimeline[4] {
	{
	},
	{
		.activateAtLifetimeFraction = 0.20f, .replacementPalette = kSmokeSoftLayerFadeInPalette,
		.sumOfReplacementChanceForThisSegment = 3.5f,
		.allowIncreasingOpacity = true,
	},
	{
		.activateAtLifetimeFraction = 0.35f,
	},
	{
		.activateAtLifetimeFraction = 0.85f,
		.sumOfDropChanceForThisSegment = 3.0f
	}
};

static const SimulatedHullsSystem::ColorChangeTimelineNode kSmokeHullHardLayerColorChangeTimeline[4] {
	{
	},
	{
		.activateAtLifetimeFraction = 0.20f, .replacementPalette = kSmokeHardLayerFadeInPalette,
		.sumOfReplacementChanceForThisSegment = 3.5f,
		.allowIncreasingOpacity = true,
	},
	{
		.activateAtLifetimeFraction = 0.35f,
	},
	{
		.activateAtLifetimeFraction = 0.85f,
		.sumOfDropChanceForThisSegment = 3.0f
	}
};

static const uint8_t kSmokeHullNoColorChangeVertexColor[4] { 127, 127, 127, 0 };
static const uint16_t kSmokeHullNoColorChangeIndices[] { 28, 100, 101, 103, 104, 106, 157, 158 };

static SimulatedHullsSystem::CloudMeshProps g_smokeOuterLayerCloudMeshProps[2] {
	{
		.alphaScaleLifespan              = { .initial = 0.0f, .fadedIn = 1.25f, .fadedOut = 1.0f },
		.radiusLifespan                  = { .initial = 0.0f, .fadedIn = 16.0f, .fadedOut = 0.0f },
		.tessLevelShiftForMinVertexIndex = -3,
		.tessLevelShiftForMaxVertexIndex = -2,
		.applyRotation                   = true,
	},
	{
		.alphaScaleLifespan              = { .initial = 0.0f, .fadedIn = 1.0f, .fadedOut = 1.0f },
		.radiusLifespan                  = { .initial = 0.0f, .fadedIn = 24.0f, .fadedOut = 0.0f },
		.tessLevelShiftForMinVertexIndex = -1,
		.tessLevelShiftForMaxVertexIndex = -1,
		.shiftFromDefaultLevelToHide     = -2,
	},
};

// Caution: Specifying the full range of vertices would not allow the cloud to be rendered due to reaching the limit

static SimulatedHullsSystem::CloudMeshProps g_fireInnerCloudMeshProps {
	.alphaScaleLifespan              = { .initial = 0.0f, .fadedIn = 1.0f, .fadedOut = 0.0f },
	.radiusLifespan                  = { .initial = 0.0f, .fadedIn = 3.0f, .fadedOut = 1.0f },
	.tessLevelShiftForMinVertexIndex = -1,
	.tessLevelShiftForMaxVertexIndex = -0,
	.shiftFromDefaultLevelToHide     = -1,
};

static SimulatedHullsSystem::CloudMeshProps g_fireOuterCloudMeshProps {
	.alphaScaleLifespan              = { .initial = 0.0f, .fadedIn = 0.75f, .fadedOut = 0.75f },
	.radiusLifespan                  = { .initial = 0.0f, .fadedIn = 6.0f, .fadedOut = 3.0f },
	.tessLevelShiftForMinVertexIndex = -1,
	.tessLevelShiftForMaxVertexIndex = -0,
	.shiftFromDefaultLevelToHide     = -1,
};

static SimulatedHullsSystem::AppearanceRules g_fireInnerCloudAppearanceRules = SimulatedHullsSystem::SolidAppearanceRules {};
static SimulatedHullsSystem::AppearanceRules g_fireOuterCloudAppearanceRules = SimulatedHullsSystem::SolidAppearanceRules {};

void TransientEffectsSystem::spawnExplosionHulls( const float *fireOrigin, const float *smokeOrigin, float radius ) {
	// 250 for radius of 64
	// TODO: Make radius affect hulls
	constexpr float lightRadiusScale = 1.0f / 64.0f;
	allocLightEffect( m_lastTime, fireOrigin, vec3_origin, 0.0f, 400u, LightLifespan {
		.colorLifespan = {
			.initial  = { 1.0f, 0.9f, 0.7f },
			.fadedIn  = { 1.0f, 0.8f, 0.5f },
			.fadedOut = { 1.0f, 0.5f, 0.0f },
		},
		.radiusLifespan = {
			.fadedIn = 250.0f * radius * lightRadiusScale,
			.finishFadingInAtLifetimeFrac = 0.15f,
			.startFadingOutAtLifetimeFrac = 0.75f,
		},
	});

	SimulatedHullsSystem *const hullsSystem = &cg.simulatedHullsSystem;

	float fireHullScale;
	unsigned fireHullTimeout;
	std::span<const SimulatedHullsSystem::HullLayerParams> fireHullLayerParams;
	if( cg_explosionsSmoke->integer ) {
		fireHullScale       = 1.40f;
		fireHullTimeout     = 550;
		fireHullLayerParams = kFireHullParams.lightHullLayerParams;
	} else {
		fireHullScale       = 1.55f;
		fireHullTimeout     = 500;
		fireHullLayerParams = kFireHullParams.darkHullLayerParams;
	}

	if( auto *const hull = hullsSystem->allocFireHull( m_lastTime, fireHullTimeout ) ) {
		hullsSystem->setupHullVertices( hull, fireOrigin, fireHullScale, fireHullLayerParams );
		assert( !hull->layers[0].useDrawOnTopHack );
        hull->vertexViewDotFade          = SimulatedHullsSystem::ViewDotFade::FadeOutContour;
        hull->layers[0].useDrawOnTopHack = true;
        hull->layers[0].overrideHullFade = SimulatedHullsSystem::ViewDotFade::NoFade;
        hull->layers[1].overrideHullFade = SimulatedHullsSystem::ViewDotFade::NoFade;

		// We have to update material references due to invalidation upon restarts
		g_fireInnerCloudMeshProps.material = cgs.media.shaderFireHullParticle;
		g_fireOuterCloudMeshProps.material = cgs.media.shaderFireHullParticle;

		g_fireInnerCloudAppearanceRules = SimulatedHullsSystem::SolidAndCloudAppearanceRules {
			.cloudRules = SimulatedHullsSystem::CloudAppearanceRules {
				.spanOfMeshProps = { &g_fireInnerCloudMeshProps, 1 },
			}
		};

		g_fireOuterCloudAppearanceRules = SimulatedHullsSystem::SolidAndCloudAppearanceRules {
			.cloudRules = SimulatedHullsSystem::CloudAppearanceRules {
				.spanOfMeshProps = { &g_fireOuterCloudMeshProps, 1 },
			}
		};

		hull->layers[0].overrideAppearanceRules                   = &g_fireInnerCloudAppearanceRules;
		hull->layers[hull->numLayers - 1].overrideAppearanceRules = &g_fireOuterCloudAppearanceRules;
	}

	if( cg_explosionsWave->integer ) {
		const vec4_t waveColor { 1.0f, 1.0f, 1.0f, 0.05f };
		if( auto *const hull = hullsSystem->allocWaveHull( m_lastTime, 250 ) ) {
			hullsSystem->setupHullVertices( hull, fireOrigin, waveColor, 500.0f, 50.0f );
		}
	}

	if( smokeOrigin ) {
        if( auto *const hull = hullsSystem->allocToonSmokeHull( m_lastTime, 2500 ) ) {
            hullsSystem->setupHullVertices( hull, smokeOrigin, fireHullScale, fireHullLayerParams );
            Com_Printf("bbbbb");
            assert( !hull->layers[0].useDrawOnTopHack );
            hull->vertexViewDotFade          = SimulatedHullsSystem::ViewDotFade::FadeOutContour;
            hull->layers[0].useDrawOnTopHack = true;
            hull->layers[0].overrideHullFade = SimulatedHullsSystem::ViewDotFade::NoFade;
            hull->layers[1].overrideHullFade = SimulatedHullsSystem::ViewDotFade::NoFade;

            // We have to update material references due to invalidation upon restarts
            g_fireInnerCloudMeshProps.material = cgs.media.shaderFireHullParticle;
            g_fireOuterCloudMeshProps.material = cgs.media.shaderFireHullParticle;

            g_fireInnerCloudAppearanceRules = SimulatedHullsSystem::SolidAndCloudAppearanceRules {
                    .cloudRules = SimulatedHullsSystem::CloudAppearanceRules {
                            .spanOfMeshProps = { &g_fireInnerCloudMeshProps, 1 },
                    }
            };

            g_fireOuterCloudAppearanceRules = SimulatedHullsSystem::SolidAndCloudAppearanceRules {
                    .cloudRules = SimulatedHullsSystem::CloudAppearanceRules {
                            .spanOfMeshProps = { &g_fireOuterCloudMeshProps, 1 },
                    }
            };

            hull->layers[0].overrideAppearanceRules                   = &g_fireInnerCloudAppearanceRules;
            hull->layers[hull->numLayers - 1].overrideAppearanceRules = &g_fireOuterCloudAppearanceRules;
        }
        Com_Printf("aaaaa");
    }
        /*
		g_smokeOuterLayerCloudMeshProps[0].material = cgs.media.shaderSmokeHullHardParticle;
		g_smokeOuterLayerCloudMeshProps[1].material = cgs.media.shaderSmokeHullSoftParticle;

		SimulatedHullsSystem::CloudAppearanceRules cloudRulesMsvcWorkaround {
			.spanOfMeshProps = g_smokeOuterLayerCloudMeshProps,
		};

		// Cannot be declared with a static lifetime due to material dependency
		const TransientEffectsSystem::SmokeHullParams spawnSmokeHullParams[] {
			{
				.speed               = { .mean = 85.0f, .spread = 15.0f },
				.archimedesAccel     = {
					.top    = { .initial = +125.0f, .fadedIn = +100.0f, .fadedOut = 0.0f, .startFadingOutAtLifetimeFrac = 0.5f, },
					.bottom = { .initial = 0.0f, .fadedIn = +75.0f, .fadedOut = +75.0f, .startFadingOutAtLifetimeFrac = 0.5f, },
				},
				.xyExpansionAccel    = {
					.top    = { .initial = 0.0f, .fadedIn = +95.0f, .fadedOut = 0.0f },
					.bottom = { .initial = 0.0f, .fadedIn = -45.0f, .fadedOut = -55.0f },
				},
				.viewDotFade         = SimulatedHullsSystem::ViewDotFade::FadeOutCenterCubic,
				.zFade               = SimulatedHullsSystem::ZFade::FadeOutBottom,
				.colorChangeTimeline = kSmokeHullHardLayerColorChangeTimeline,
			},
			{
				.speed               = { .mean = 85.0f, .spread = 15.0f },
				.archimedesAccel     = {
					.top    = { .initial = +130.0f, .fadedIn = +110.0f, .fadedOut = 0.0f, .startFadingOutAtLifetimeFrac = 0.5f },
					.bottom = { .initial = 0.0f, .fadedIn = +75.0f, .fadedOut = 75.0f, .startFadingOutAtLifetimeFrac = 0.5f },
				},
				.xyExpansionAccel    = {
					.top    = { .initial = 0.0f, .fadedIn = +105.0f, .fadedOut = 0.0f },
					.bottom = { .initial = 0.0f, .fadedIn = -40.0f, .fadedOut = -50.0f },
				},
				.viewDotFade         = SimulatedHullsSystem::ViewDotFade::FadeOutContour,
				.zFade               = SimulatedHullsSystem::ZFade::FadeOutBottom,
				.colorChangeTimeline = kSmokeHullSoftLayerColorChangeTimeline,
				.appearanceRules     = SimulatedHullsSystem::SolidAndCloudAppearanceRules {
					.cloudRules      = cloudRulesMsvcWorkaround,
				},
			},
		};

		for( const SmokeHullParams &hullSpawnParams: spawnSmokeHullParams ) {
			spawnSmokeHull( m_lastTime, smokeOrigin, hullSpawnParams );
		}
	}*/

	if( cg_explosionsClusters->integer ) {
		std::span<const SimulatedHullsSystem::HullLayerParams> clusterHullLayerParams;
		float minSpawnerSpeed, maxSpawnerSpeed;
		unsigned maxSpawnedClusters;
		if( cg_explosionsSmoke->integer ) {
			clusterHullLayerParams = ::kFireHullParams.lightClusterHullLayerParams;
			minSpawnerSpeed = 105.0f, maxSpawnerSpeed = 125.0f;
			maxSpawnedClusters = 7;
		} else {
			clusterHullLayerParams = ::kFireHullParams.darkClusterHullLayerParams;
			minSpawnerSpeed = 115.0f, maxSpawnerSpeed = 135.0f;
			maxSpawnedClusters = 10;
		}

		unsigned numSpawnedClusters = 0;
		unsigned oldDirNum          = 0;

		while( numSpawnedClusters < maxSpawnedClusters ) {
			const unsigned dirNum  = m_rng.nextBoundedFast( std::size( kPredefinedDirs ) );
			const float *randomDir = kPredefinedDirs[dirNum];
			// Just check against the last directory so this converges faster
			if( DotProduct( randomDir, kPredefinedDirs[oldDirNum] ) > 0.7f ) {
				continue;
			}

			oldDirNum = dirNum;
			numSpawnedClusters++;

			const auto spawnDelay = ( fireHullTimeout / 4 ) + m_rng.nextBoundedFast( fireHullTimeout / 4 );
			auto *const effect = allocDelayedEffect( m_lastTime, fireOrigin, spawnDelay, ConcentricHullSpawnRecord {
				.layerParams = clusterHullLayerParams,
				.scale       = m_rng.nextFloat( 0.25f, 0.37f ) * fireHullScale,
				.timeout     = fireHullTimeout / 3,
				.allocMethod = (ConcentricHullSpawnRecord::AllocMethod)&SimulatedHullsSystem::allocFireClusterHull,
				.vertexViewDotFade         = SimulatedHullsSystem::ViewDotFade::FadeOutContour,
				.useLayer0DrawOnTopHack    = true,
				.overrideLayer0ViewDotFade = SimulatedHullsSystem::ViewDotFade::NoFade,
			});

			const float randomSpeed = m_rng.nextFloat( minSpawnerSpeed, maxSpawnerSpeed );
			VectorScale( randomDir, randomSpeed, effect->velocity );
			effect->simulation = DelayedEffect::SimulateMovement;
		}
	}
}
/*
void TransientEffectsSystem::spawnSmokeHull( int64_t currTime, const float *origin, const SmokeHullParams &params ) {
	if( auto *const hull = cg.simulatedHullsSystem.allocSmokeHull( currTime, 2000 ) ) {
		hull->archimedesTopAccel      = params.archimedesAccel.top;
		hull->archimedesBottomAccel   = params.archimedesAccel.bottom;
		hull->xyExpansionTopAccel     = params.xyExpansionAccel.top;
		hull->xyExpansionBottomAccel  = params.xyExpansionAccel.bottom;

		hull->colorChangeTimeline      = params.colorChangeTimeline;
		hull->noColorChangeIndices     = kSmokeHullNoColorChangeIndices;
		hull->noColorChangeVertexColor = kSmokeHullNoColorChangeVertexColor;

		hull->expansionStartAt = m_lastTime + 125;

		hull->tesselateClosestLod = true;
		hull->leprNextLevelColors = true;
		hull->applyVertexDynLight = true;
		hull->vertexViewDotFade   = params.viewDotFade;
		hull->vertexZFade         = params.zFade;

		const vec4_t initialSmokeColor { 0.0f, 0.0f, 0.0f, 0.03f };
		cg.simulatedHullsSystem.setupHullVertices( hull, origin, initialSmokeColor,
												   params.speed.mean, params.speed.spread, params.appearanceRules );
	}
}*/

void TransientEffectsSystem::spawnCartoonHitEffect( const float *origin, const float *dir, int damage ) {
	if( cg_cartoonHitEffect->integer ) {
		float radius = 0.0f;
		shader_s *material = nullptr;
		if( damage > 64 ) {
			// OUCH!
			std::tie( material, radius ) = std::make_pair( cgs.media.shaderCartoonHit3, 24.0f );
		} else if( damage > 50 ) {
			// POW!
			std::tie( material, radius ) = std::make_pair( cgs.media.shaderCartoonHit, 19.0f );
		} else if( damage > 38 ) {
			// SPLITZOW!
			std::tie( material, radius ) = std::make_pair( cgs.media.shaderCartoonHit2, 15.0f );
		}

		if( material ) {
			// TODO:
			vec3_t localDir;
			if( !VectorLength( dir ) ) {
				VectorNegate( &cg.view.axis[AXIS_FORWARD], localDir );
			} else {
				VectorNormalize2( dir, localDir );
			}

			vec3_t spriteOrigin;
			// Move effect a bit up from player
			VectorCopy( origin, spriteOrigin );
			spriteOrigin[2] += ( playerbox_stand_maxs[2] - playerbox_stand_mins[2] ) + 1.0f;

			EntityEffect *effect = addSpriteEntityEffect( material, spriteOrigin, radius, 700u );
			effect->entity.rotation = 0.0f;
			// TODO: Add a correct sampling of random sphere points as a random generator method
			for( unsigned i = 0; i < 3; ++i ) {
				effect->velocity[i] = m_rng.nextFloat( -10.0f, +10.0f );
			}
		}
	}
}

void TransientEffectsSystem::spawnBleedingVolumeEffect( const float *origin, const float *dir, int damage,
														const float *bloodColor, unsigned duration ) {
	if( auto *hull = cg.simulatedHullsSystem.allocWaveHull( m_lastTime, duration ) ) {
		vec3_t hullOrigin;
		constexpr float offset = -32.0f;
		VectorMA( origin, offset, dir, hullOrigin );

		float speed, speedSpreadFrac;
		bool tesselateClosestLod = true;
		SimulatedHullsSystem::ViewDotFade viewDotFade;
		// TODO: Avoid hardcoding damage values
		// TODO: Lift the code to EffectsSystemFacade, get rid of the separate TransientEffectsSystem
		if( damage < 20 ) {
			speed               = 50.0f;
			speedSpreadFrac     = 0.18f;
			tesselateClosestLod = false;
			viewDotFade         = SimulatedHullsSystem::ViewDotFade::FadeOutCenterLinear;
		} else if( damage < 40 ) {
			speed           = 80.0f;
			speedSpreadFrac = 0.27f;
			viewDotFade     = SimulatedHullsSystem::ViewDotFade::FadeOutCenterLinear;
		} else if( damage < 70 ) {
			speed           = 110.0f;
			speedSpreadFrac = 0.39f;
			viewDotFade     = SimulatedHullsSystem::ViewDotFade::FadeOutCenterQuadratic;
		} else {
			speed           = 140.0f;
			speedSpreadFrac = 0.50f;
			viewDotFade     = SimulatedHullsSystem::ViewDotFade::FadeOutCenterCubic;
		}

		const vec4_t hullColor { bloodColor[0], bloodColor[1], bloodColor[2], 0.5f };
		cg.simulatedHullsSystem.setupHullVertices( hull, hullOrigin, hullColor, speed, speedSpreadFrac * speed );
		hull->vertexViewDotFade   = viewDotFade;
		hull->tesselateClosestLod = tesselateClosestLod;
		hull->minFadedOutAlpha    = 0.1f;
	}
}

void TransientEffectsSystem::spawnElectroboltHitEffect( const float *origin, const float *dir, const float *decalColor,
														const float *energyColor, bool spawnDecal ) {
	spawnElectroboltLikeHitEffect( origin, dir, decalColor, energyColor, cgs.media.modElectroBoltWallHit, spawnDecal );
}

void TransientEffectsSystem::spawnInstagunHitEffect( const float *origin, const float *dir, const float *decalColor,
													 const float *energyColor, bool spawnDecal ) {
	spawnElectroboltLikeHitEffect( origin, dir, decalColor, energyColor, cgs.media.modInstagunWallHit, spawnDecal );
}

void TransientEffectsSystem::spawnElectroboltLikeHitEffect( const float *origin, const float *dir, 
															const float *decalColor, const float *energyColor, 
															model_s *model, bool spawnDecal ) {
	if( spawnDecal ) {
		EntityEffect *entityEffect = addModelEntityEffect( model, origin, dir, 600u );
		VectorScale( decalColor, 255.0f, entityEffect->entity.shaderRGBA );
	}

	allocLightEffect( m_lastTime, origin, dir, 4.0f, 250u, LightLifespan {
		.colorLifespan = {
			.initial  = { 1.0f, 1.0f, 1.0f },
			.fadedIn  = { energyColor[0], energyColor[1], energyColor[2] },
			.fadedOut = { energyColor[0], energyColor[1], energyColor[2] },
			.finishFadingInAtLifetimeFrac = 0.10f,
		},
		.radiusLifespan = {
			.fadedIn = 144.0f,
			.finishFadingInAtLifetimeFrac = 0.10f,
		},
	});

	if( cg_explosionsWave->integer ) {
		if( auto *hull = cg.simulatedHullsSystem.allocWaveHull( m_lastTime, 200 ) ) {
			const vec4_t hullColor { energyColor[0], energyColor[1], energyColor[2], 0.075f };
			cg.simulatedHullsSystem.setupHullVertices( hull, origin, hullColor, 750.0f, 100.0f );
		}
		if( auto *hull = cg.simulatedHullsSystem.allocWaveHull( m_lastTime, 200 ) ) {
			const vec4_t hullColor { energyColor[0], energyColor[1], energyColor[2], 0.1f };
			cg.simulatedHullsSystem.setupHullVertices( hull, origin, hullColor, 125.0f, 50.0f );
		}
	}
}

void TransientEffectsSystem::spawnPlasmaImpactEffect( const float *origin, const float *dir ) {
	EntityEffect *const entityEffect = addModelEntityEffect( cgs.media.modPlasmaExplosion, origin, dir, 300u );
	entityEffect->scaleLifespan = {
		.initial                      = 0.0f,
		.fadedIn                      = 2.5f,
		.fadedOut                     = 2.5f,
		.finishFadingInAtLifetimeFrac = 0.1f,
	};

	allocLightEffect( m_lastTime, origin, dir, 4.0f, 200, LightLifespan {
		.colorLifespan = {
			.initial  = { 1.0f, 1.0f, 1.0f },
			.fadedIn  = { 0.0f, 1.0f, 0.3f },
			.fadedOut = { 0.0f, 0.7f, 0.0f },
		},
		.radiusLifespan = { .fadedIn = 96.0f, },
	});

	if( cg_explosionsWave->integer ) {
		if( auto *hull = cg.simulatedHullsSystem.allocWaveHull( m_lastTime, 175 ) ) {
			const vec4_t hullColor { colorGreen[0], colorGreen[1], colorGreen[2], 0.05f };
			cg.simulatedHullsSystem.setupHullVertices( hull, origin, hullColor, 300.0f, 75.0f );
		}
	}
}

static const byte_vec4_t kBlastHullLayer0ReplacementPalette[] {
	asByteColor( 0.7f, 0.7f, 0.5f, 0.1f ),
	asByteColor( 0.7f, 0.7f, 0.4f, 0.1f ),
	asByteColor( 0.7f, 0.7f, 0.4f, 0.1f ),
};

static const byte_vec4_t kBlastHullLayer1ReplacementPalette[] {
	asByteColor( 0.7f, 0.7f, 0.5f, 0.1f ),
	asByteColor( 0.7f, 0.7f, 0.4f, 0.1f ),
	asByteColor( 0.7f, 0.6f, 0.4f, 0.1f ),
};

static const byte_vec4_t kBlastHullLayer2ReplacementPalette[] {
	asByteColor( 0.7f, 0.7f, 0.3f, 0.1f ),
	asByteColor( 0.7f, 0.7f, 0.4f, 0.1f ),
	asByteColor( 0.7f, 0.5f, 0.4f, 0.1f ),
};

static const byte_vec4_t kBlastHullDecayPalette[] {
	asByteColor( 0.7f, 0.7f, 0.4f, 0.05f ),
	asByteColor( 0.7f, 0.3f, 0.2f, 0.05f ),
	asByteColor( 0.7f, 0.4f, 0.1f, 0.05f )
};

static const SimulatedHullsSystem::ColorChangeTimelineNode kBlastHullLayer0ColorChangeTimeline[3] {
	{
		.sumOfDropChanceForThisSegment = 0.1f,
	},
	{
		.activateAtLifetimeFraction = 0.33f, .replacementPalette = kBlastHullLayer0ReplacementPalette,
		.sumOfDropChanceForThisSegment = 0.1f, .sumOfReplacementChanceForThisSegment = 0.3f,
	},
	{
		.activateAtLifetimeFraction = 0.67f, .replacementPalette = kBlastHullDecayPalette,
		.sumOfDropChanceForThisSegment = 1.5f, .sumOfReplacementChanceForThisSegment = 1.0f,
	}
};

static const SimulatedHullsSystem::ColorChangeTimelineNode kBlastHullLayer1ColorChangeTimeline[3] {
	{
	},
	{
		.activateAtLifetimeFraction = 0.33f, .replacementPalette = kBlastHullLayer1ReplacementPalette,
		.sumOfDropChanceForThisSegment = 0.1f, .sumOfReplacementChanceForThisSegment = 0.5f,
	},
	{
		.activateAtLifetimeFraction = 0.67f, .replacementPalette = kBlastHullDecayPalette,
		.sumOfDropChanceForThisSegment = 1.5f, .sumOfReplacementChanceForThisSegment = 1.0f,
	}
};

static const SimulatedHullsSystem::ColorChangeTimelineNode kBlastHullLayer2ColorChangeTimeline[3] {
	{
	},
	{
		.activateAtLifetimeFraction = 0.33f, .replacementPalette = kBlastHullLayer2ReplacementPalette,
		.sumOfDropChanceForThisSegment = 0.1f, .sumOfReplacementChanceForThisSegment = 0.7f,
	},
	{
		.activateAtLifetimeFraction = 0.67f, .replacementPalette = kBlastHullDecayPalette,
		.sumOfDropChanceForThisSegment = 1.5f, .sumOfReplacementChanceForThisSegment = 1.0f,
	}
};

static const SimulatedHullsSystem::HullLayerParams kBlastHullLayerParams[3] {
	{
		.speed = 30.0f, .finalOffset = 5.0f,
		.speedSpikeChance = 0.10f, .minSpeedSpike = 5.0f, .maxSpeedSpike = 20.0f,
		.biasAlongChosenDir = 15.0f,
		.baseInitialColor = { 0.9f, 1.0f, 0.6f, 1.0f },
		.bulgeInitialColor = { 0.9f, 1.0f, 1.0f, 1.0f },
		.colorChangeTimeline = kBlastHullLayer0ColorChangeTimeline
	},
	{
		.speed = 40.0f, .finalOffset = 2.5f,
		.speedSpikeChance = 0.08f, .minSpeedSpike = 5.0f, .maxSpeedSpike = 20.0f,
		.biasAlongChosenDir = 15.0f,
		.baseInitialColor = { 1.0f, 0.7f, 0.4f, 1.0f },
		.bulgeInitialColor = { 1.0f, 0.8f, 0.5f, 1.0f },
		.colorChangeTimeline = kBlastHullLayer1ColorChangeTimeline
	},
	{
		.speed = 50.0f, .finalOffset = 0.0f,
		.speedSpikeChance = 0.08f, .minSpeedSpike = 5.0f, .maxSpeedSpike = 20.0f,
		.biasAlongChosenDir = 15.0f,
		.baseInitialColor = { 0.7, 0.6f, 0.4f, 0.7f },
		.bulgeInitialColor = { 1.0f, 0.7f, 0.4f, 0.7f },
		.colorChangeTimeline = kBlastHullLayer2ColorChangeTimeline
	},
};

static SimulatedHullsSystem::CloudMeshProps g_blastHullCloudMeshProps {
	.alphaScaleLifespan          = { .initial = 0.0f, .fadedIn = 0.1f, .fadedOut = 0.1f },
	.radiusLifespan              = { .initial = 0.0f, .fadedIn = 4.0f, .fadedOut = 1.0f },
	.shiftFromDefaultLevelToHide = -1,
};

void TransientEffectsSystem::spawnGunbladeBlastImpactEffect( const float *origin, const float *dir ) {
	allocLightEffect( m_lastTime, origin, dir, 8.0f, 350u, LightLifespan {
		.colorLifespan = {
			.initial  = { 1.0f, 1.0f, 0.5f },
			.fadedIn  = { 1.0f, 0.8f, 0.3f },
			.fadedOut = { 0.5f, 0.7f, 0.3f },
		},
		.radiusLifespan = { .fadedIn = 144.0f, },
	});

	const vec3_t hullOrigin { origin[0] + 8.0f * dir[0], origin[1] + 8.0f * dir[1], origin[2] + 8.0f * dir[2] };

	if( auto *hull = cg.simulatedHullsSystem.allocBlastHull( m_lastTime, 450 ) ) {
		cg.simulatedHullsSystem.setupHullVertices( hull, hullOrigin, 1.25f, kBlastHullLayerParams );
		assert( !hull->layers[0].useDrawOnTopHack );
		hull->vertexViewDotFade          = SimulatedHullsSystem::ViewDotFade::FadeOutContour;
		hull->layers[0].useDrawOnTopHack = true;
		hull->layers[0].overrideHullFade = SimulatedHullsSystem::ViewDotFade::NoFade;

		g_blastHullCloudMeshProps.material = cgs.media.shaderBlastHullParticle;

		SimulatedHullsSystem::CloudAppearanceRules cloudRulesMsvcWorkaround {
			.spanOfMeshProps = { &g_blastHullCloudMeshProps, 1 },
		};

		hull->appearanceRules = SimulatedHullsSystem::SolidAndCloudAppearanceRules {
			.cloudRules = cloudRulesMsvcWorkaround,
		};
	}

	if( cg_explosionsWave->integer ) {
		if( auto *hull = cg.simulatedHullsSystem.allocWaveHull( m_lastTime, 200 ) ) {
			const vec4_t waveHullColor { 1.0f, 0.9f, 0.6f, 0.05f };
			cg.simulatedHullsSystem.setupHullVertices( hull, hullOrigin, waveHullColor, 300.0f, 30.0f );
		}
	}
}

void TransientEffectsSystem::spawnGunbladeBladeImpactEffect( const float *origin, const float *dir ) {
	(void)addModelEntityEffect( cgs.media.modBladeWallHit, origin, dir, 300u );
	// TODO: Add light when hitting metallic surfaces?
}

void TransientEffectsSystem::spawnBulletImpactModel( const float *origin, const float *dir ) {
	EntityEffect *effect = addModelEntityEffect( cgs.media.modBladeWallExplo, origin, dir, 33u );
	effect->scaleLifespan = {
		.initial                      = 0.0f,
		.fadedIn                      = 0.3f,
		.fadedOut                     = 0.3f,
		.finishFadingInAtLifetimeFrac = 0.1f,
		.startFadingOutAtLifetimeFrac = 0.3f,
	};
}

void TransientEffectsSystem::spawnPelletImpactModel( const float *origin, const float *dir ) {
	EntityEffect *effect = addModelEntityEffect( cgs.media.modBladeWallExplo, origin, dir, 108u );
	effect->scaleLifespan = {
		.initial                      = 0.0f,
		.fadedIn                      = 0.3f,
		.fadedOut                     = 0.2f,
		.finishFadingInAtLifetimeFrac = 0.05f,
		.startFadingOutAtLifetimeFrac = 0.35f,
	};
}

void TransientEffectsSystem::spawnImpactRing( const float *origin, const float *axisDir, unsigned timeout,
											  const ValueLifespan &scaleLifespan, const ValueLifespan &alphaLifespan ) {
	assert( std::fabs( VectorLengthFast( axisDir ) - 1.0f ) < 0.1f );

	QuadPoly::OrientedSpriteRules appearanceRules {};
	VectorCopy( axisDir, appearanceRules.axis );
	MakeNormalVectors( axisDir, appearanceRules.axis + 3, appearanceRules.axis + 6 );

	PolyEffect *const effect     = allocPolyEffect( m_lastTime, timeout );
	effect->poly.material        = cgs.media.shaderImpactRing;
	effect->poly.appearanceRules = appearanceRules;

	VectorMA( origin, 4.0f, axisDir, effect->poly.origin );

	effect->scaleLifespan   = scaleLifespan;
	effect->alphaLifespan   = alphaLifespan;
	effect->scaleMultiplier = m_rng.nextFloat( 0.9f, 1.1f );
}

static const ValueLifespan kBulletImpactRingScaleLifespan {
	.initial = 0.0f, .fadedIn = 72.0f, .fadedOut = 96.0f,
	.finishFadingInAtLifetimeFrac = 0.08f, .startFadingOutAtLifetimeFrac = 0.10f,
};

static const ValueLifespan kBulletImpactRingAlphaLifespan {
	.initial = 1.0f, .fadedIn = 0.25f, .fadedOut = 0.0f,
	.finishFadingInAtLifetimeFrac = 0.15f, .startFadingOutAtLifetimeFrac = 0.17f,
};

void TransientEffectsSystem::spawnBulletLikeImpactRing( const float *origin, const float *axisDir ) {
	spawnImpactRing( origin, axisDir, 250, kBulletImpactRingScaleLifespan, kBulletImpactRingAlphaLifespan );
}

static const ValueLifespan kWaterImpactRingScaleLifespan {
	.initial = 0.0f, .fadedIn = 56.0f, .fadedOut = 72.0f,
	.finishFadingInAtLifetimeFrac = 0.45f, .startFadingOutAtLifetimeFrac = 0.47f,
};

static const ValueLifespan kWaterImpactRingAlphaLifespan {
	.initial = 0.0f, .fadedIn = 0.33f, .fadedOut = 0.0f,
	.finishFadingInAtLifetimeFrac = 0.15f, .startFadingOutAtLifetimeFrac = 0.20f,
};

void TransientEffectsSystem::spawnWaterImpactRing( const float *origin, const float *axisDir ) {
	spawnImpactRing( origin, axisDir, 575, kWaterImpactRingScaleLifespan, kWaterImpactRingAlphaLifespan );
}

void TransientEffectsSystem::addDelayedParticleEffect( unsigned delay, ParticleFlockBin bin,
													   const ConicalFlockParams &flockParams,
													   const Particle::AppearanceRules &appearanceRules ) {
	allocDelayedEffect( m_lastTime, flockParams.origin, delay, ConicalFlockSpawnRecord {
		.flockParams = flockParams, .appearanceRules = appearanceRules, .bin = bin
	});
}

void TransientEffectsSystem::addDelayedParticleEffect( unsigned delay, ParticleFlockBin bin,
													   const EllipsoidalFlockParams &flockParams,
													   const Particle::AppearanceRules &appearanceRules ) {
	allocDelayedEffect( m_lastTime, flockParams.origin, delay, EllipsoidalFlockSpawnRecord {
		.flockParams = flockParams, .appearanceRules = appearanceRules, .bin = bin
	});
}

void TransientEffectsSystem::addDelayedImpactRosetteEffect( unsigned delay,
															const PolyEffectsSystem::ImpactRosetteParams &params ) {
	allocDelayedEffect( m_lastTime, vec3_origin, delay, ImpactRosetteSpawnRecord { .params = params } );
}

void TransientEffectsSystem::spawnDustImpactEffect( const float *origin, const float *dir, float radius ) {
	vec3_t axis1, axis2;
	PerpendicularVector( axis2, dir );
	CrossProduct( dir, axis2, axis1 );

	VectorNormalize( axis1 ), VectorNormalize( axis2 );

	float angle = 0.0f;
	constexpr const int count = 12;
	const float speed = 0.67f * radius;
	const float angleStep = (float)M_TWOPI * Q_Rcp( (float)count );
	for( int i = 0; i < count; ++i ) {
		const float scale1 = std::sin( angle ), scale2 = std::cos( angle );
		angle += angleStep;

		vec3_t velocity { 0.0f, 0.0f, 0.0f };
		VectorMA( velocity, speed * scale1, axis1, velocity );
		VectorMA( velocity, speed * scale2, axis2, velocity );

		EntityEffect *effect  = addSpriteEntityEffect( cgs.media.shaderSmokePuff2, origin, 10.0f, 700u );
		effect->alphaLifespan = { .initial  = 0.25f, .fadedIn  = 0.25f, .fadedOut = 0.0f };
		effect->scaleLifespan = { .initial  = 0.0f, .fadedIn  = 0.33f, .fadedOut = 0.0f };
		VectorCopy( velocity, effect->velocity );
	}
}

void TransientEffectsSystem::spawnDashEffect( const float *origin, const float *dir ) {
	// Model orientation/streching hacks
	vec3_t angles;
	VecToAngles( dir, angles );
	angles[1] += 270.0f;
	EntityEffect *effect = addModelEntityEffect( cgs.media.modDash, origin, dir, 700u );
	AnglesToAxis( angles, effect->entity.axis );
	// Scale Z
	effect->entity.axis[2 * 3 + 2] *= 2.0f;
	effect->scaleLifespan = {
		.initial                      = 0.0f,
		.fadedIn                      = 0.15f,
		.fadedOut                     = 0.15f,
		.finishFadingInAtLifetimeFrac = 0.12f,
	};
}

auto TransientEffectsSystem::addModelEntityEffect( model_s *model, const float *origin, const float *dir,
												   unsigned duration ) -> EntityEffect * {
	EntityEffect *const effect = allocEntityEffect( m_lastTime, duration );

	std::memset( &effect->entity, 0, sizeof( entity_s ) );
	effect->entity.rtype = RT_MODEL;
	effect->entity.renderfx = RF_NOSHADOW;
	effect->entity.model = model;
	effect->entity.customShader = nullptr;
	effect->entity.shaderTime = m_lastTime;
	effect->entity.scale = 0.0f;
	effect->entity.rotation = (float)m_rng.nextBounded( 360 );

	VectorSet( effect->entity.shaderRGBA, 255, 255, 255 );

	NormalVectorToAxis( dir, &effect->entity.axis[0] );
	VectorCopy( origin, effect->entity.origin );

	return effect;
}

auto TransientEffectsSystem::addSpriteEntityEffect( shader_s *material, const float *origin, float radius,
													unsigned duration ) -> EntityEffect * {
	EntityEffect *const effect = allocEntityEffect( m_lastTime, duration );

	std::memset( &effect->entity, 0, sizeof( entity_s ) );
	effect->entity.rtype = RT_SPRITE;
	effect->entity.renderfx = RF_NOSHADOW;
	effect->entity.radius = radius;
	effect->entity.customShader = material;
	effect->entity.shaderTime = m_lastTime;
	effect->entity.scale = 0.0f;
	effect->entity.rotation = (float)m_rng.nextBounded( 360 );

	VectorSet( effect->entity.shaderRGBA, 255, 255, 255 );

	Matrix3_Identity( effect->entity.axis );
	VectorCopy( origin, effect->entity.origin );

	return effect;
}

auto TransientEffectsSystem::allocEntityEffect( int64_t currTime, unsigned duration ) -> EntityEffect * {
	void *mem = m_entityEffectsAllocator.allocOrNull();
	if( !mem ) [[unlikely]] {
		// TODO: Prioritize effects so unimportant ones get evicted first
		EntityEffect *oldestEffect = nullptr;
		// TODO: Choose by nearest timeout/lifetime fraction?
		int64_t oldestSpawnTime = std::numeric_limits<int64_t>::max();
		for( EntityEffect *effect = m_entityEffectsHead; effect; effect = effect->next ) {
			if( oldestSpawnTime > effect->spawnTime ) {
				oldestSpawnTime = effect->spawnTime;
				oldestEffect = effect;
			}
		}
		assert( oldestEffect );
		wsw::unlink( oldestEffect, &m_entityEffectsHead );
		oldestEffect->~EntityEffect();
		mem = oldestEffect;
	}

	assert( duration >= 16 && duration <= std::numeric_limits<uint16_t>::max() );

	auto *effect = new( mem )EntityEffect;
	effect->duration  = duration;
	effect->spawnTime = currTime;

	wsw::link( effect, &m_entityEffectsHead );
	return effect;
}

// TODO: Generalize!!!
auto TransientEffectsSystem::allocPolyEffect( int64_t currTime, unsigned duration ) -> PolyEffect * {
	void *mem = m_polyEffectsAllocator.allocOrNull();
	if( !mem ) [[unlikely]] {
		// TODO: Prioritize effects so unimportant ones get evicted first
		PolyEffect *oldestEffect = nullptr;
		// TODO: Choose by nearest timeout/lifetime fraction?
		int64_t oldestSpawnTime = std::numeric_limits<int64_t>::max();
		for( PolyEffect *effect = m_polyEffectsHead; effect; effect = effect->next ) {
			if( oldestSpawnTime > effect->spawnTime ) {
				oldestSpawnTime = effect->spawnTime;
				oldestEffect = effect;
			}
		}
		assert( oldestEffect );
		wsw::unlink( oldestEffect, &m_polyEffectsHead );
		oldestEffect->~PolyEffect();
		mem = oldestEffect;
	}

	auto *effect = new( mem )PolyEffect;
	effect->duration  = duration;
	effect->spawnTime = currTime;

	wsw::link( effect, &m_polyEffectsHead );
	return effect;
}

auto TransientEffectsSystem::allocLightEffect( int64_t currTime, const float *origin, const float *offset,
											   float offsetScale, unsigned duration,
											   LightLifespan &&lightLifespan ) -> LightEffect * {
	void *mem = m_lightEffectsAllocator.allocOrNull();
	// TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Generalize
	if( !mem ) [[unlikely]] {
		// TODO: Prioritize effects so unimportant ones get evicted first
		LightEffect *oldestEffect = nullptr;
		// TODO: Choose by nearest timeout/lifetime fraction?
		int64_t oldestSpawnTime = std::numeric_limits<int64_t>::max();
		for( LightEffect *effect = m_lightEffectsHead; effect; effect = effect->next ) {
			if( oldestSpawnTime > effect->spawnTime ) {
				oldestSpawnTime = effect->spawnTime;
				oldestEffect = effect;
			}
		}
		assert( oldestEffect );
		wsw::unlink( oldestEffect, &m_lightEffectsHead );
		oldestEffect->~LightEffect();
		mem = oldestEffect;
	}

	auto *effect          = new( mem )LightEffect;
	effect->duration      = duration;
	effect->spawnTime     = currTime;
	effect->lightLifespan = std::forward<LightLifespan>( lightLifespan );
	VectorMA( origin, offsetScale, offset, effect->origin );

	wsw::link( effect, &m_lightEffectsHead );
	return effect;
}

auto TransientEffectsSystem::allocDelayedEffect( int64_t currTime, const float *origin, unsigned delay,
												 DelayedEffect::SpawnRecord &&spawnRecord ) -> DelayedEffect * {
	void *mem = m_delayedEffectsAllocator.allocOrNull();
	// TODO!!!!!!!!!!!!
	if( !mem ) {
		DelayedEffect *oldestEffect = nullptr;
		// TODO: Choose by nearest timeout/lifetime fraction?
		int64_t oldestSpawnTime = std::numeric_limits<int64_t>::max();
		for( DelayedEffect *effect = m_delayedEffectsHead; effect; effect = effect->next ) {
			if( oldestSpawnTime > effect->spawnTime ) {
				oldestSpawnTime = effect->spawnTime;
				oldestEffect = effect;
			}
		}
		assert( oldestEffect );
		wsw::unlink( oldestEffect, &m_delayedEffectsHead );
		oldestEffect->~DelayedEffect();
		mem = oldestEffect;
	}

	auto *effect = new( mem )DelayedEffect { .spawnRecord = std::move_if_noexcept( spawnRecord ) };
	effect->spawnTime  = currTime;
	effect->spawnDelay = delay;
	VectorCopy( origin, effect->origin );

	wsw::link( effect, &m_delayedEffectsHead );
	return effect;
}

void TransientEffectsSystem::unlinkAndFreeEntityEffect( EntityEffect *effect ) {
	wsw::unlink( effect, &m_entityEffectsHead );
	effect->~EntityEffect();
	m_entityEffectsAllocator.free( effect );
}

void TransientEffectsSystem::unlinkAndFreePolyEffect( PolyEffect *effect ) {
	wsw::unlink( effect, &m_polyEffectsHead );
	effect->~PolyEffect();
	m_polyEffectsAllocator.free( effect );
}

void TransientEffectsSystem::unlinkAndFreeLightEffect( LightEffect *effect ) {
	wsw::unlink( effect, &m_lightEffectsHead );
	effect->~LightEffect();
	m_lightEffectsAllocator.free( effect );
}

void TransientEffectsSystem::unlinkAndFreeDelayedEffect( DelayedEffect *effect ) {
	wsw::unlink( effect, &m_delayedEffectsHead );
	effect->~DelayedEffect();
	m_delayedEffectsAllocator.free( effect );
}

void TransientEffectsSystem::simulateFrameAndSubmit( int64_t currTime, DrawSceneRequest *request ) {
	// Limit the time step
	const float timeDeltaSeconds = 1e-3f * (float)wsw::min<int64_t>( 33, currTime - m_lastTime );

	simulateDelayedEffects( currTime, timeDeltaSeconds );
	simulateEntityEffectsAndSubmit( currTime, timeDeltaSeconds, request );
	simulatePolyEffectsAndSubmit( currTime, timeDeltaSeconds, request );
	simulateLightEffectsAndSubmit( currTime, timeDeltaSeconds, request );

	m_lastTime = currTime;
}

void TransientEffectsSystem::simulateEntityEffectsAndSubmit( int64_t currTime, float timeDeltaSeconds,
															 DrawSceneRequest *request ) {
	const model_s *const dashModel = cgs.media.modDash;
	const float backlerp = 1.0f - cg.lerpfrac;

	EntityEffect *nextEffect = nullptr;
	for( EntityEffect *__restrict effect = m_entityEffectsHead; effect; effect = nextEffect ) {
		nextEffect = effect->next;

		if( effect->spawnTime + effect->duration <= currTime ) [[unlikely]] {
			unlinkAndFreeEntityEffect( effect );
			continue;
		}

		// Dash model hacks
		if( effect->entity.model == dashModel ) [[unlikely]] {
			float *const zScale = effect->entity.axis + ( 2 * 3 ) + 2;
			*zScale -= 4.0f * timeDeltaSeconds;
			if( *zScale < 0.01f ) {
				unlinkAndFreeEntityEffect( effect );
				continue;
			}
		}

		const auto lifetimeMillis = (unsigned)( currTime - effect->spawnTime );
		assert( lifetimeMillis < effect->duration );
		const float lifetimeFrac = (float)lifetimeMillis * Q_Rcp( (float)effect->duration );

		vec3_t moveVec;
		VectorScale( effect->velocity, timeDeltaSeconds, moveVec );
		VectorAdd( effect->entity.origin, moveVec, effect->entity.origin );

		effect->entity.backlerp      = backlerp;
		effect->entity.scale         = effect->scaleLifespan.getValueForLifetimeFrac( lifetimeFrac );
		effect->entity.shaderRGBA[3] = (uint8_t)( 255 * effect->alphaLifespan.getValueForLifetimeFrac( lifetimeFrac ) );
		effect->entity.shaderTime    = currTime;

		request->addEntity( &effect->entity );
	}
}

void TransientEffectsSystem::simulatePolyEffectsAndSubmit( int64_t currTime, float timeDeltaSeconds,
														   DrawSceneRequest *request ) {

	PolyEffect *nextEffect = nullptr;
	for( PolyEffect *__restrict effect = m_polyEffectsHead; effect; effect = nextEffect ) {
		nextEffect = effect->next;

		if( effect->spawnTime + effect->duration <= currTime ) [[unlikely]] {
			unlinkAndFreePolyEffect( effect );
			continue;
		}

		vec3_t moveVec;
		VectorScale( effect->velocity, timeDeltaSeconds, moveVec );
		VectorAdd( effect->poly.origin, moveVec, effect->poly.origin );

		const auto lifetimeMillis = (unsigned)( currTime - effect->spawnTime );
		assert( lifetimeMillis < effect->duration );
		const float lifetimeFrac = (float)lifetimeMillis * Q_Rcp( (float)effect->duration );

		effect->poly.halfExtent = effect->scaleLifespan.getValueForLifetimeFrac( lifetimeFrac ) * effect->scaleMultiplier;

		const float colorAlpha = effect->alphaLifespan.getValueForLifetimeFrac( lifetimeFrac );
		// std::variant<> interface is awful
		if( auto *beamRules = std::get_if<QuadPoly::ViewAlignedBeamRules>( &effect->poly.appearanceRules ) ) {
			beamRules->fromColor[3] = colorAlpha;
			beamRules->toColor[3]   = colorAlpha;
		} else if( auto *spriteRules = std::get_if<QuadPoly::ViewAlignedSpriteRules>( &effect->poly.appearanceRules ) ) {
			spriteRules->color[3] = colorAlpha;
		} else if( auto *orientedRules = std::get_if<QuadPoly::OrientedSpriteRules>( &effect->poly.appearanceRules ) ) {
			orientedRules->color[3] = colorAlpha;
		}

		request->addPoly( &effect->poly );
	}
}

void TransientEffectsSystem::simulateLightEffectsAndSubmit( int64_t currTime, float timeDeltaSeconds,
															DrawSceneRequest *request ) {
	LightEffect *nextEffect = nullptr;
	for( LightEffect *__restrict effect = m_lightEffectsHead; effect; effect = nextEffect ) {
		nextEffect = effect->next;

		if( effect->spawnTime + effect->duration <= currTime ) [[unlikely]] {
			unlinkAndFreeLightEffect( effect );
			continue;
		}

		const float lifetimeFrac = (float)( currTime - effect->spawnTime ) * Q_Rcp( (float)effect->duration );

		float radius, color[3];
		effect->lightLifespan.getRadiusAndColorForLifetimeFrac( lifetimeFrac, &radius, color );

		if( radius >= 1.0f ) [[likely]] {
			request->addLight( effect->origin, radius, 0.0f, color );
		}
	}

	// TODO: Add and use a bulk submission of lights
}

void TransientEffectsSystem::simulateDelayedEffects( int64_t currTime, float timeDeltaSeconds ) {
	DelayedEffect *nextEffect = nullptr;
	for( DelayedEffect *effect = m_delayedEffectsHead; effect; effect = nextEffect ) { nextEffect = effect->next;
		bool isSpawningPossible = true;
		if( effect->simulation == DelayedEffect::SimulateMovement ) {
			// TODO: Normalize angles each step?
			VectorMA( effect->angles, timeDeltaSeconds, effect->angularVelocity, effect->angles );

			vec3_t idealOrigin;
			VectorMA( effect->origin, timeDeltaSeconds, effect->velocity, idealOrigin );
			idealOrigin[2] += 0.5f * effect->gravity * timeDeltaSeconds * timeDeltaSeconds;

			effect->velocity[2] += effect->gravity * timeDeltaSeconds;

			trace_t trace;
			CM_TransformedBoxTrace( cl.cms, &trace, effect->origin, idealOrigin, vec3_origin, vec3_origin,
									nullptr, MASK_SOLID | MASK_WATER, nullptr, nullptr, 0 );

			if( trace.fraction == 1.0f ) {
				VectorCopy( idealOrigin, effect->origin );
			} else if ( !( trace.startsolid | trace.allsolid ) ) {
				vec3_t velocityDir;
				VectorCopy( effect->velocity, velocityDir );
				if( const float squareSpeed = VectorLengthSquared( velocityDir ); squareSpeed > 1.0f ) {
					const float rcpSpeed = Q_RSqrt( squareSpeed );
					VectorScale( velocityDir, rcpSpeed, velocityDir );
					vec3_t reflectedDir;
					VectorReflect( velocityDir, trace.plane.normal, 0.0f, reflectedDir );
					addRandomRotationToDir( reflectedDir, &m_rng, 0.9f );
					const float newSpeed = effect->restitution * squareSpeed * rcpSpeed;
					VectorScale( reflectedDir, newSpeed, effect->velocity );
					// This is not correct but is sufficient
					VectorAdd( trace.endpos, trace.plane.normal, effect->origin );
				}
			}
			// Don't spawn in solid or while contacting solid
			isSpawningPossible = trace.fraction == 1.0f && !trace.startsolid;
		}

		const int64_t triggerAt = effect->spawnTime + effect->spawnDelay;
		if( triggerAt <= currTime ) {
			if( isSpawningPossible ) {
				spawnDelayedEffect( effect );
				unlinkAndFreeDelayedEffect( effect );
			} else if( triggerAt + 25 < currTime ) {
				// If the "grace" period for getting out of solid has expired
				unlinkAndFreeDelayedEffect( effect );
			}
		}
	}
}

void TransientEffectsSystem::spawnDelayedEffect( DelayedEffect *effect ) {
	struct Visitor {
		const int64_t m_lastTime;
		DelayedEffect *const m_effect;
		
		void operator()( const RegularHullSpawnRecord &record ) const {
			auto method = record.allocMethod;
			if( auto *hull = ( cg.simulatedHullsSystem.*method )( m_lastTime, record.timeout ) ) {
				cg.simulatedHullsSystem.setupHullVertices( hull, m_effect->origin, record.color,
														   record.speed, record.speedSpread );
				hull->colorChangeTimeline = record.colorChangeTimeline;
				hull->tesselateClosestLod = record.tesselateClosestLod;
				hull->leprNextLevelColors = record.lerpNextLevelColors;
				hull->applyVertexDynLight = record.applyVertexDynLight;
				hull->vertexViewDotFade   = record.vertexViewDotFade;
				hull->vertexZFade         = record.vertexZFade;
			}
		}
		void operator()( const ConcentricHullSpawnRecord &record ) const {
			auto method = record.allocMethod;
			if( auto *hull = ( cg.simulatedHullsSystem.*method )( m_lastTime, record.timeout ) ) {
				cg.simulatedHullsSystem.setupHullVertices( hull, m_effect->origin, record.scale,
														   record.layerParams );
				assert( !hull->layers[0].useDrawOnTopHack );
				hull->vertexViewDotFade          = record.vertexViewDotFade;
				hull->layers[0].useDrawOnTopHack = record.useLayer0DrawOnTopHack;
				hull->layers[0].overrideHullFade = record.overrideLayer0ViewDotFade;
			}
		}
		void operator()( const ConicalFlockSpawnRecord &record ) const {
			ConicalFlockParams modifiedFlockParams;
			const ConicalFlockParams *flockParams   = &record.flockParams;
			const Particle::AppearanceRules &arules = record.appearanceRules;
			if( m_effect->simulation != TransientEffectsSystem::DelayedEffect::NoSimulation ) {
				modifiedFlockParams = record.flockParams;
				VectorCopy( m_effect->origin, modifiedFlockParams.origin );
				VectorClear( modifiedFlockParams.offset );
				AngleVectors( m_effect->angles, modifiedFlockParams.dir, nullptr, nullptr );
				flockParams = &modifiedFlockParams;
			}
			// TODO: "using enum"
			using Pfb = ParticleFlockBin;
			switch( record.bin ) {
				case Pfb::Small: cg.particleSystem.addSmallParticleFlock( arules, *flockParams ); break;
				case Pfb::Medium: cg.particleSystem.addMediumParticleFlock( arules, *flockParams ); break;
				case Pfb::Large: cg.particleSystem.addLargeParticleFlock( arules, *flockParams ); break;
			}
		}
		void operator()( const EllipsoidalFlockSpawnRecord &record ) const {
			EllipsoidalFlockParams modifiedFlockParams;
			const EllipsoidalFlockParams *flockParams = &record.flockParams;
			const Particle::AppearanceRules &arules   = record.appearanceRules;
			if( m_effect->simulation != TransientEffectsSystem::DelayedEffect::NoSimulation ) {
				modifiedFlockParams = record.flockParams;
				VectorCopy( m_effect->origin, modifiedFlockParams.origin );
				VectorClear( modifiedFlockParams.offset );
				if( modifiedFlockParams.stretchScale != 1.0f ) {
					AngleVectors( m_effect->angles, modifiedFlockParams.stretchDir, nullptr, nullptr );
				}
				flockParams = &modifiedFlockParams;
			}
			// TODO: "using enum"
			using Pfb = ParticleFlockBin;
			switch( record.bin ) {
				case Pfb::Small: cg.particleSystem.addSmallParticleFlock( arules, *flockParams ); break;
				case Pfb::Medium: cg.particleSystem.addMediumParticleFlock( arules, *flockParams ); break;
				case Pfb::Large: cg.particleSystem.addLargeParticleFlock( arules, *flockParams ); break;
			}
		}
		void operator()( const ImpactRosetteSpawnRecord &record ) const {
			cg.polyEffectsSystem.spawnImpactRosette( PolyEffectsSystem::ImpactRosetteParams { record.params } );
		}
	};

	std::visit( Visitor { .m_lastTime = m_lastTime, .m_effect = effect }, effect->spawnRecord );
}