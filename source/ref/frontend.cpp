/*
Copyright (C) 2007 Victor Luchits

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

#include "local.h"
#include "frontend.h"
#include "program.h"
#include "materiallocal.h"
#include "../common/singletonholder.h"

/*
* R_Set2DMode
*
* Note that this sets the viewport to size of the active framebuffer.
*/
void R_Set2DMode( bool enable ) {
	wsw::ref::Frontend::instance()->set2DMode( enable );
}

void RF_Set2DScissor( int x, int y, int w, int h ) {
	wsw::ref::Frontend::instance()->set2DScissor( x, y, w, h );
}

namespace wsw::ref {

auto Frontend::getDefaultFarClip( const refdef_s *fd ) const -> float {
	float dist;

	if( fd->rdflags & RDF_NOWORLDMODEL ) {
		dist = 1024;
	} else if( rsh.worldModel && rsh.worldBrushModel->globalfog ) {
		dist = rsh.worldBrushModel->globalfog->shader->fog_dist;
	} else {
		// TODO: Restore computations of world bounds
		dist = (float)( 1 << 16 );
	}

	return wsw::max( Z_NEAR, dist ) + Z_BIAS;
}

auto Frontend::getFogForBounds( const StateForCamera *stateForCamera, const float *mins, const float *maxs ) -> mfog_t * {
	if( stateForCamera->refdef.rdflags & RDF_NOWORLDMODEL ) {
		return nullptr;
	}
	if( !rsh.worldModel || !rsh.worldBrushModel || !rsh.worldBrushModel->numfogs ) {
		return nullptr;
	}
	if( stateForCamera->renderFlags & RF_SHADOWMAPVIEW ) {
		return nullptr;
	}
	if( rsh.worldBrushModel->globalfog ) {
		return rsh.worldBrushModel->globalfog;
	}
	for( unsigned i = 0; i < rsh.worldBrushModel->numfogs; i++ ) {
		mfog_t *const fog = rsh.worldBrushModel->fogs;
		if( fog->shader ) {
			if( BoundsIntersect( mins, maxs, fog->mins, fog->maxs ) ) {
				return fog;
			}
		}
	}

	return nullptr;
}

auto Frontend::getFogForSphere( const StateForCamera *stateForCamera, const vec3_t centre, const float radius ) -> mfog_t * {
	vec3_t mins, maxs;
	for( unsigned i = 0; i < 3; i++ ) {
		mins[i] = centre[i] - radius;
		maxs[i] = centre[i] + radius;
	}
	return getFogForBounds( stateForCamera, mins, maxs );
}

auto Frontend::createDrawSceneRequest( const refdef_t &refdef ) -> DrawSceneRequest * {
	R_ClearSkeletalCache();

	assert( m_drawSceneRequestHolder.empty() );
	return new( m_drawSceneRequestHolder.unsafe_grow_back() )DrawSceneRequest( refdef );
}

void Frontend::submitDrawSceneRequest( DrawSceneRequest *request ) {
	assert( request == m_drawSceneRequestHolder.data() );
	renderScene( request, &request->m_refdef );
	m_drawSceneRequestHolder.clear();
}

Frontend::Frontend() {
	if( Q_CPU_FEATURE_SSE41 & Sys_GetProcessorFeatures() ) {
		m_collectVisibleWorldLeavesArchMethod          = &Frontend::collectVisibleWorldLeavesSse41;
		m_collectVisibleOccludersArchMethod            = &Frontend::collectVisibleOccludersSse41;
		m_buildFrustaOfOccludersArchMethod             = &Frontend::buildFrustaOfOccludersSse41;
		m_cullLeavesByOccludersArchMethod              = &Frontend::cullLeavesByOccludersSse41;
		m_cullSurfacesInVisLeavesByOccludersArchMethod = &Frontend::cullSurfacesInVisLeavesByOccludersSse41;
		m_cullEntriesWithBoundsArchMethod              = &Frontend::cullEntriesWithBoundsSse41;
		m_cullEntryPtrsWithBoundsArchMethod            = &Frontend::cullEntryPtrsWithBoundsSse41;
	} else {
		m_collectVisibleWorldLeavesArchMethod          = &Frontend::collectVisibleWorldLeavesSse2;
		m_collectVisibleOccludersArchMethod            = &Frontend::collectVisibleOccludersSse2;
		m_buildFrustaOfOccludersArchMethod             = &Frontend::buildFrustaOfOccludersSse2;
		m_cullLeavesByOccludersArchMethod              = &Frontend::cullLeavesByOccludersSse2;
		m_cullSurfacesInVisLeavesByOccludersArchMethod = &Frontend::cullSurfacesInVisLeavesByOccludersSse2;
		m_cullEntriesWithBoundsArchMethod              = &Frontend::cullEntriesWithBoundsSse2;
		m_cullEntryPtrsWithBoundsArchMethod            = &Frontend::cullEntryPtrsWithBoundsSse2;
	}
}

alignas( 32 ) static SingletonHolder<Frontend> sceneInstanceHolder;

void Frontend::init() {
	sceneInstanceHolder.init();
}

void Frontend::shutdown() {
	sceneInstanceHolder.shutdown();
}

Frontend *Frontend::instance() {
	return sceneInstanceHolder.instance();
}

void Frontend::initVolatileAssets() {
	m_coronaShader = MaterialCache::instance()->loadDefaultMaterial( "$corona"_asView, SHADER_TYPE_CORONA );
}

void Frontend::destroyVolatileAssets() {
	m_coronaShader = nullptr;
}

void Frontend::dynLightDirForOrigin( const vec_t *origin, float radius, vec3_t dir, vec3_t diffuseLocal, vec3_t ambientLocal ) {
}

}

Scene::Scene() {
	m_worldent = m_localEntities.unsafe_grow_back();
	memset( m_worldent, 0, sizeof( entity_t ) );
	m_worldent->rtype = RT_MODEL;
	m_worldent->number = 0;
	m_worldent->model = rsh.worldModel;
	m_worldent->scale = 1.0f;
	Matrix3_Identity( m_worldent->axis );
	m_entities.push_back( m_worldent );

	m_polyent = m_localEntities.unsafe_grow_back();
	memset( m_polyent, 0, sizeof( entity_t ) );
	m_polyent->rtype = RT_MODEL;
	m_polyent->number = 1;
	m_polyent->model = nullptr;
	m_polyent->scale = 1.0f;
	Matrix3_Identity( m_polyent->axis );
	m_entities.push_back( m_polyent );
}

void DrawSceneRequest::addEntity( const entity_t *ent ) {
	if( !m_entities.full() && ent ) [[likely]] {
		entity_t *added = nullptr;

		if( ent->rtype == RT_MODEL ) {
			if( const model_t *__restrict model = ent->model ) [[likely]] {
				if( model->type == mod_alias ) {
					m_aliasModelEntities.push_back( *ent );
					added = std::addressof( m_aliasModelEntities.back() );
				} else if( model->type == mod_skeletal ) {
					m_skeletalModelEntities.push_back( *ent );
					added = std::addressof( m_skeletalModelEntities.back() );
				} else if( model->type == mod_brush ) {
					m_brushModelEntities.push_back( *ent );
					added = std::addressof( m_brushModelEntities.back() );
				}
			} else {
				m_nullModelEntities.push_back( *ent );
				added = std::addressof( m_nullModelEntities.back() );
			}
		} else if( ent->rtype == RT_SPRITE ) [[unlikely]] {
			m_spriteEntities.push_back( *ent );
			added = std::addressof( m_spriteEntities.back() );
			// simplifies further checks
			added->model = nullptr;
		} else if( ent->rtype == RT_PORTALSURFACE ) [[unlikely]] {
			m_portalSurfaceEntities.push_back( *ent );
			added = std::addressof( m_portalSurfaceEntities.back() );
			added->model = nullptr;
		}

		if( added ) {
			if( r_outlines_scale->value <= 0 ) {
				added->outlineHeight = 0;
			}

			if( !r_lerpmodels->integer ) {
				added->backlerp = 0;
			}

			if( added->renderfx & RF_ALPHAHACK ) {
				if( added->shaderRGBA[3] == 255 ) {
					added->renderfx &= ~RF_ALPHAHACK;
				}
			}

			m_entities.push_back( added );
			added->number = m_entities.size() - 1;

			// add invisible fake entity for depth write
			// TODO: This should belong to the CGame code
			if( ( added->renderfx & ( RF_WEAPONMODEL | RF_ALPHAHACK ) ) == ( RF_WEAPONMODEL | RF_ALPHAHACK ) ) {
				entity_t tent = *ent;
				tent.renderfx &= ~RF_ALPHAHACK;
				tent.renderfx |= RF_NOCOLORWRITE | RF_NOSHADOW;
				addEntity( &tent );
			}
		}
	}
}

void DrawSceneRequest::addLight( const float *origin, float programRadius, float coronaRadius, float r, float g, float b ) {
	assert( ( r >= 0.0f && r >= 0.0f && b >= 0.0f ) && ( r > 0.0f || g > 0.0f || b > 0.0f ) );
	if( !m_dynamicLights.full() ) [[likely]] {
		if( const int cvarValue = r_dynamiclight->integer ) [[likely]] {
			const bool hasProgramLight = programRadius > 0.0f && ( cvarValue & 1 ) != 0;
			const bool hasCoronaLight = coronaRadius > 0.0f && ( cvarValue & 2 ) != 0;
			if( hasProgramLight | hasCoronaLight ) [[likely]] {
				// Bounds are used for culling and also for applying lights to particles
				const float maxRadius = wsw::max( programRadius, coronaRadius );
				m_dynamicLights.emplace_back( DynamicLight {
					.origin          = { origin[0], origin[1], origin[2] },
					.programRadius   = programRadius,
					.coronaRadius    = coronaRadius,
					.maxRadius       = maxRadius,
					.color           = { r, g, b },
					.mins            = { origin[0] - maxRadius, origin[1] - maxRadius, origin[2] - maxRadius, 0.0f },
					.maxs            = { origin[0] + maxRadius, origin[1] + maxRadius, origin[2] + maxRadius, 1.0f },
					.hasProgramLight = hasProgramLight,
					.hasCoronaLight  = hasCoronaLight
				});
			}
		}
	}
}

void DrawSceneRequest::addParticles( const float *mins, const float *maxs,
									 const Particle::AppearanceRules &appearanceRules,
									 const Particle *particles, unsigned numParticles ) {
	assert( numParticles <= kMaxParticlesInAggregate );
	if( !m_particles.full() ) [[likely]] {
		m_particles.emplace_back( ParticlesAggregate {
			.particles       = particles,
			.appearanceRules = appearanceRules,
			.mins            = { mins[0], mins[1], mins[2], mins[3] },
			.maxs            = { maxs[0], maxs[1], maxs[2], maxs[3] },
			.numParticles    = numParticles
		});
	}
}

void DrawSceneRequest::addDynamicMesh( const DynamicMesh *mesh ) {
	if( !m_dynamicMeshes.full() ) [[likely]] {
		m_dynamicMeshes.push_back( mesh );
	}
}

void DrawSceneRequest::addCompoundDynamicMesh( const float *mins, const float *maxs,
											   const DynamicMesh **parts, unsigned numParts,
											   const float *meshOrderDesignators ) {
	assert( numParts <= kMaxPartsInCompoundMesh );
	if( !m_compoundDynamicMeshes.full() ) [[likely]] {
		m_compoundDynamicMeshes.emplace_back( CompoundDynamicMesh {
			.cullMins             = { mins[0], mins[1], mins[2], mins[3] },
			.cullMaxs             = { maxs[0], maxs[1], maxs[2], maxs[3] },
			.parts                = parts,
			.meshOrderDesignators = meshOrderDesignators,
			.numParts             = numParts,
		});
	}
}