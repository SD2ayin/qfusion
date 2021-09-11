/*
Copyright (C) 1997-2001 Id Software, Inc.
Copyright (C) 2002-2007 Victor Luchits

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

// r_model.c -- model loading and caching

#include "local.h"
#include "iqm.h"
#include "../qcommon/qcommon.h"

#include <algorithm>

typedef struct {
	unsigned number;
	msurface_t *surf;
} msortedSurface_t;

void Mod_LoadAliasMD3Model( model_t *mod, model_t *parent, void *buffer, bspFormatDesc_t *unused );
void Mod_LoadSkeletalModel( model_t *mod, model_t *parent, void *buffer, bspFormatDesc_t *unused );
void Mod_LoadQ3BrushModel( model_t *mod, model_t *parent, void *buffer, bspFormatDesc_t *format );

static void R_InitMapConfig( const char *model );
static void R_FinishMapConfig( const model_t *mod );

static uint8_t mod_novis[MAX_MAP_LEAFS / 8];

#define MAX_MOD_KNOWN   512 * MOD_MAX_LODS
static model_t mod_known[MAX_MOD_KNOWN];
static int mod_numknown;
static int modfilelen;
static bool mod_isworldmodel;
model_t *r_prevworldmodel;
static mapconfig_t *mod_mapConfigs;

static const modelFormatDescr_t mod_supportedformats[] =
{
	// Quake III Arena .md3 models
	{ IDMD3HEADER, 4, NULL, MOD_MAX_LODS, ( const modelLoader_t )Mod_LoadAliasMD3Model },

	// Skeletal models
	{ IQM_MAGIC, sizeof( IQM_MAGIC ), NULL, MOD_MAX_LODS, ( const modelLoader_t )Mod_LoadSkeletalModel },

	// Q3-alike .bsp models
	{ "*", 4, q3BSPFormats, 0, ( const modelLoader_t )Mod_LoadQ3BrushModel },

	// trailing NULL
	{ NULL, 0, NULL, 0, NULL }
};

//===============================================================================

/*
* Mod_PointInLeaf
*/
mleaf_t *Mod_PointInLeaf( vec3_t p, model_t *model ) {
	mbrushmodel_t *bmodel;

	if( !model || !( bmodel = ( mbrushmodel_t * )model->extradata ) || !bmodel->nodes ) {
		Com_Error( ERR_DROP, "Mod_PointInLeaf: bad model" );
		return NULL;
	}

	const auto *__restrict nodes = bmodel->nodes;

	int nodeNum = 0;
	do {
		const auto *__restrict node = nodes + nodeNum;
		const auto *__restrict plane = &node->plane;
		nodeNum = node->children[PlaneDiff( p, plane ) < 0];
	} while( nodeNum >= 0 );

	return bmodel->leafs + ( -1 - nodeNum );
}

/*
* Mod_ClusterVS
*/
static inline uint8_t *Mod_ClusterVS( int cluster, dvis_t *vis ) {
	if( cluster < 0 || !vis ) {
		return mod_novis;
	}
	return ( (uint8_t *)vis->data + cluster * vis->rowsize );
}

/*
* Mod_ClusterPVS
*/
uint8_t *Mod_ClusterPVS( int cluster, model_t *model ) {
	return Mod_ClusterVS( cluster, ( ( mbrushmodel_t * )model->extradata )->pvs );
}

//===============================================================================

/*
* Mod_CreateVisLeafs
*/
static void Mod_CreateVisLeafs( model_t *mod ) {
	unsigned i, j;
	unsigned count, numVisLeafs;
	unsigned numVisSurfaces, numFragmentSurfaces;
	mleaf_t *leaf;
	msurface_t *surf;
	mbrushmodel_t *loadbmodel = ( ( mbrushmodel_t * )mod->extradata );

	count = loadbmodel->numleafs;
	loadbmodel->visleafs = (mleaf_t **)Q_malloc( (count + 1) * sizeof( *loadbmodel->visleafs ) );
	memset( loadbmodel->visleafs, 0, (count + 1) * sizeof( *loadbmodel->visleafs ) );

	numVisLeafs = 0;
	for( i = 0; i < count; i++ ) {
		numVisSurfaces = numFragmentSurfaces = 0;

		leaf = loadbmodel->leafs + i;
		if( leaf->cluster < 0 || !leaf->numVisSurfaces ) {
			leaf->visSurfaces = NULL;
			leaf->numVisSurfaces = 0;
			leaf->fragmentSurfaces = NULL;
			leaf->numFragmentSurfaces = 0;
			leaf->fragmentSurfaces = NULL;
			continue;
		}

		for( j = 0; j < leaf->numVisSurfaces; j++ ) {
			unsigned surfNum;

			surfNum = leaf->visSurfaces[j];
			surf = loadbmodel->surfaces + surfNum;

			if( R_SurfPotentiallyVisible( surf ) ) {
				leaf->visSurfaces[numVisSurfaces++] = surfNum;
				if( R_SurfPotentiallyFragmented( surf ) ) {
					leaf->fragmentSurfaces[numFragmentSurfaces++] = surfNum;
				}
			}
		}

		leaf->numVisSurfaces = numVisSurfaces;
		leaf->numFragmentSurfaces = numFragmentSurfaces;

		if( !numVisSurfaces ) {
			//out->cluster = -1;
			continue;
		}

		loadbmodel->visleafs[numVisLeafs++] = leaf;
	}

	loadbmodel->visleafs[numVisLeafs] = NULL;
	loadbmodel->numvisleafs = numVisLeafs;
}

/*
* Mod_CalculateAutospriteBounds
*
* Make bounding box of an autosprite surf symmetric and enlarges it
* to account for rotation along the longest axis.
*/
static void Mod_CalculateAutospriteBounds( msurface_t *surf ) {
	int j;
	int l_axis, s1_axis, s2_axis;
	vec_t dist, max_dist;
	vec_t radius[3];
	vec3_t centre;
	vec_t *mins = surf->mins, *maxs = surf->maxs;

	// find the longest axis
	l_axis = 2;
	max_dist = -9999999;
	for( j = 0; j < 3; j++ ) {
		dist = maxs[j] - mins[j];
		if( dist > max_dist ) {
			l_axis = j;
			max_dist = dist;
		}

		// make the bbox symmetrical
		radius[j] = dist * 0.5;
		centre[j] = ( maxs[j] + mins[j] ) * 0.5;
		mins[j] = centre[j] - radius[j];
		maxs[j] = centre[j] + radius[j];
	}

	// shorter axis
	s1_axis = ( l_axis + 1 ) % 3;
	s2_axis = ( l_axis + 2 ) % 3;

	// enlarge the bounding box, accouting for rotation along the longest axis
	maxs[s1_axis] = std::max( maxs[s1_axis], centre[s1_axis] + radius[s2_axis] );
	maxs[s2_axis] = std::max( maxs[s2_axis], centre[s2_axis] + radius[s1_axis] );

	mins[s1_axis] = std::min( mins[s1_axis], centre[s1_axis] - radius[s2_axis] );
	mins[s2_axis] = std::min( mins[s2_axis], centre[s2_axis] - radius[s1_axis] );
}

/*
* Mod_FinishFaces
*/
static void Mod_FinishFaces( model_t *mod ) {
	unsigned int i, j;
	shader_t *shader;
	mbrushmodel_t *loadbmodel = ( ( mbrushmodel_t * )mod->extradata );

	for( i = 0; i < loadbmodel->numsurfaces; i++ ) {
		vec_t *vert;
		msurface_t *surf;
		const mesh_t *mesh;

		surf = loadbmodel->surfaces + i;
		mesh = &surf->mesh;
		shader = surf->shader;

		if( !R_SurfPotentiallyVisible( surf ) ) {
			continue;
		}

		// calculate bounding box of a surface
		vert = mesh->xyzArray[0];
		VectorCopy( vert, surf->mins );
		VectorCopy( vert, surf->maxs );
		for( j = 1, vert += 4; j < mesh->numVerts; j++, vert += 4 ) {
			AddPointToBounds( vert, surf->mins, surf->maxs );
		}

		// foliage surfaces need special treatment for bounds
		if( surf->facetype == FACETYPE_FOLIAGE ) {
			vec3_t temp;

			for( j = 0, vert = &surf->instances[0][4]; j < surf->numInstances; j++, vert += 8 ) {
				VectorMA( vert, vert[3], surf->mins, temp );
				AddPointToBounds( temp, surf->mins, surf->maxs );

				VectorMA( vert, vert[3], surf->maxs, temp );
				AddPointToBounds( temp, surf->mins, surf->maxs );
			}
		}

		// handle autosprites
		if( shader->flags & SHADER_AUTOSPRITE ) {
			// handle autosprites as trisurfs to avoid backface culling
			surf->facetype = FACETYPE_TRISURF;

			Mod_CalculateAutospriteBounds( surf );
		}
	}
}

/*
* Mod_SetupSubmodels
*/
static void Mod_SetupSubmodels( model_t *mod ) {
	unsigned int i;
	mmodel_t *bm;
	model_t *starmod;
	mbrushmodel_t *bmodel;
	mbrushmodel_t *loadbmodel = ( ( mbrushmodel_t * )mod->extradata );

	// set up the submodels
	for( i = 0; i < loadbmodel->numsubmodels; i++ ) {
		bm = &loadbmodel->submodels[i];
		starmod = &loadbmodel->inlines[i];
		bmodel = ( mbrushmodel_t * )starmod->extradata;

		memcpy( starmod, mod, sizeof( model_t ) );
		if( i ) {
			memcpy( bmodel, loadbmodel, sizeof( mbrushmodel_t ) );
		}

		bmodel->firstModelSurface = bm->firstModelSurface;
		bmodel->numModelSurfaces = bm->numModelSurfaces;

		bmodel->firstModelDrawSurface = bm->firstModelDrawSurface;
		bmodel->numModelDrawSurfaces = bm->numModelDrawSurfaces;

		starmod->extradata = bmodel;
		if( i == 0 ) {
			bmodel->visleafs = loadbmodel->visleafs;
			bmodel->numvisleafs = loadbmodel->numvisleafs;
		} else {
			bmodel->visleafs = NULL;
			bmodel->numvisleafs = 0;
		}

		VectorCopy( bm->maxs, starmod->maxs );
		VectorCopy( bm->mins, starmod->mins );
		starmod->radius = bm->radius;

		if( i == 0 ) {
			*mod = *starmod;
		} else {
			bmodel->numsubmodels = 0;
		}
	}
}

#define VBO_Printf Com_DPrintf

/*
* R_CompareSurfacesByDrawSurf
*/
static int R_CompareSurfacesByDrawSurf( const void *ps1, const void *ps2 ) {
	const auto *s1 = (msortedSurface_t *)ps1;
	const auto *s2 = (msortedSurface_t *)ps2;
	if( s1->surf->drawSurf > s2->surf->drawSurf )
		return 1;
	if( s1->surf->drawSurf < s2->surf->drawSurf )
		return -1;
	return s1->surf->firstDrawSurfVert - s2->surf->firstDrawSurfVert;
}

/*
* Mod_SortModelSurfaces
*/
static void Mod_SortModelSurfaces( model_t *mod, unsigned int modnum ) {
	unsigned i, j;
	mmodel_t *bm;
	mbrushmodel_t *loadbmodel;
	unsigned numSurfaces, firstSurface;
	msurface_t *backupSurfaces;
	msortedSurface_t *sortedSurfaces;
	unsigned *map;
	unsigned lastDrawSurf;

	assert( mod );

	loadbmodel = ( ( mbrushmodel_t * )mod->extradata );
	assert( loadbmodel );

	assert( modnum >= 0 && modnum < loadbmodel->numsubmodels );
	bm = loadbmodel->submodels + modnum;

	// ignore empty models
	numSurfaces = bm->numModelSurfaces;
	firstSurface = bm->firstModelSurface;
	if( !numSurfaces ) {
		return;
	}

	map = ( unsigned * )Q_malloc( numSurfaces * sizeof( *map ) );
	sortedSurfaces = ( msortedSurface_t * )Q_malloc( numSurfaces * sizeof( *sortedSurfaces ) );
	backupSurfaces = ( msurface_t * )Q_malloc( numSurfaces * sizeof( *backupSurfaces ) );
	for( i = 0; i < numSurfaces; i++ ) {
		sortedSurfaces[i].number = i;
		sortedSurfaces[i].surf = loadbmodel->surfaces + firstSurface + i;
	}

	memcpy( backupSurfaces, loadbmodel->surfaces + firstSurface, numSurfaces * sizeof( msurface_t ) );
	qsort( sortedSurfaces, numSurfaces, sizeof( msortedSurface_t ), &R_CompareSurfacesByDrawSurf );

	for( i = 0; i < numSurfaces; i++ ) {
		map[sortedSurfaces[i].number] = i;
	}

	if( !modnum && loadbmodel->visleafs ) {
		mleaf_t *leaf, **pleaf;
		for( pleaf = loadbmodel->visleafs, leaf = *pleaf; leaf; leaf = *++pleaf ) {
			for( j = 0; j < leaf->numVisSurfaces; j++ ) {
				leaf->visSurfaces[j] = map[leaf->visSurfaces[j]];
				leaf->fragmentSurfaces[j] = map[leaf->fragmentSurfaces[j]];
			}
		}
	}

	for( i = 0; i < numSurfaces; i++ ) {
		*(loadbmodel->surfaces + firstSurface + i) = backupSurfaces[sortedSurfaces[i].number];
	}

	lastDrawSurf = loadbmodel->numDrawSurfaces + 1;
	for( i = 0; i < numSurfaces; i++ ) {
		drawSurfaceBSP_t *drawSurf;
		msurface_t *surf = loadbmodel->surfaces + firstSurface + i;

		if( !surf->drawSurf ) {
			continue;
		}

		drawSurf = &loadbmodel->drawSurfaces[surf->drawSurf - 1];

		if( lastDrawSurf != surf->drawSurf ) {
			drawSurf->numWorldSurfaces = 0;
			drawSurf->firstWorldSurface = firstSurface + i;
			lastDrawSurf = surf->drawSurf;
		}

		drawSurf->numWorldSurfaces++;
	}

	Q_free( map );
	Q_free( sortedSurfaces );
	Q_free( backupSurfaces );
}

/*
* Mod_CreateSubmodelBufferObjects
*/
static int Mod_CreateSubmodelBufferObjects( model_t *mod, unsigned int modnum, size_t *vbo_total_size ) {
	unsigned int i, j, k;
	uint8_t *visdata = NULL;
	uint8_t *areadata = NULL;
	unsigned int rowbytes, rowlongs;
	int areabytes;
	uint8_t *arearow;
	int *longrow, *longrow2;
	mmodel_t *bm;
	mbrushmodel_t *loadbmodel;
	msurface_t *surf, *surf2;
	msurface_t **surfmap;
	msurface_t **surfaces;
	unsigned numSurfaces;
	unsigned numUnmappedSurfaces;
	unsigned startDrawSurface;
	drawSurfaceBSP_t *drawSurf;
	int num_vbos;
	vattribmask_t floatVattribs;
	mesh_vbo_t *tempVBOs;
	unsigned numTempVBOs, maxTempVBOs;
	unsigned numUnmergedVBOs;

	assert( mod );

	loadbmodel = ( ( mbrushmodel_t * )mod->extradata );
	assert( loadbmodel );

	assert( modnum >= 0 && modnum < loadbmodel->numsubmodels );
	bm = loadbmodel->submodels + modnum;

	// ignore empty models
	if( !bm->numModelSurfaces ) {
		return 0;
	}

	surfmap = ( msurface_t ** )Q_malloc( bm->numModelSurfaces * sizeof( *surfmap ) );
	surfaces = ( msurface_t ** )Q_malloc( bm->numModelSurfaces * sizeof( *surfaces ) );
	numSurfaces = 0;

	numTempVBOs = 0;
	maxTempVBOs = 1024;
	tempVBOs = ( mesh_vbo_t * )Q_malloc( maxTempVBOs * sizeof( *tempVBOs ) );
	startDrawSurface = loadbmodel->numDrawSurfaces;

	bm->numModelDrawSurfaces = 0;
	bm->firstModelDrawSurface = startDrawSurface;

	if( !modnum && loadbmodel->pvs ) {
		mleaf_t *leaf, **pleaf;

		rowbytes = loadbmodel->pvs->rowsize;
		rowlongs = ( rowbytes + 3 ) / 4;
		areabytes = ( loadbmodel->numareas + 7 ) / 8;

		if( !rowbytes ) {
			return 0;
		}

		// build visibility data for each face, based on what leafs
		// this face belongs to (visible from)
		visdata = ( uint8_t * )Q_malloc( rowlongs * 4 * loadbmodel->numsurfaces );
		areadata = ( uint8_t * )Q_malloc( areabytes * loadbmodel->numsurfaces );

		for( pleaf = loadbmodel->visleafs, leaf = *pleaf; leaf; leaf = *++pleaf ) {
			for( i = 0; i < leaf->numVisSurfaces; i++ ) {
				unsigned surfnum;

				surfnum = leaf->visSurfaces[i];
				surf = loadbmodel->surfaces + surfnum;

				if( surfnum >= bm->numModelSurfaces ) {
					// some buggy maps such as aeroq2 contain visleafs that address faces from submodels...
					continue;
				}
				if( surfmap[surfnum] ) {
					continue;
				}
				surfmap[surfnum] = surf;

				surfaces[numSurfaces] = surf;

				longrow  = ( int * )( visdata + numSurfaces * rowbytes );
				longrow2 = ( int * )( Mod_ClusterPVS( leaf->cluster, mod ) );

				// merge parent leaf cluster visibility into face visibility set
				// we could probably check for duplicates here because face can be
				// shared among multiple leafs
				for( j = 0; j < rowlongs; j++ )
					longrow[j] |= longrow2[j];

				if( leaf->area >= 0 ) {
					arearow = areadata + numSurfaces * areabytes;
					arearow[leaf->area >> 3] |= ( 1 << ( leaf->area & 7 ) );
				}

				numSurfaces++;
			}
		}

		memset( surfmap, 0, bm->numModelSurfaces * sizeof( *surfmap ) );
	} else {
		// either a submodel or an unvised map
		rowbytes = 0;
		rowlongs = 0;
		visdata = NULL;
		areabytes = 0;
		areadata = NULL;

		for( i = 0, surf = loadbmodel->surfaces + bm->firstModelSurface; i < bm->numModelSurfaces; i++, surf++ ) {
			if( !R_SurfPotentiallyVisible( surf ) ) {
				continue;
			}

			surfaces[numSurfaces] = surf;
			numSurfaces++;
		}
	}

	// now linearly scan all faces for this submodel, merging them into
	// vertex buffer objects if they share shader, lightmap texture and we can render
	// them in hardware (some Q3A shaders require GLSL for that)

	// don't use half-floats for XYZ due to precision issues
	floatVattribs = VATTRIB_POSITION_BIT;
	if( mapConfig.maxLightmapSize > 1024 ) {
		// don't use half-floats for lightmaps if there's not enough precision (half mantissa is 10 bits)
		floatVattribs |= VATTRIB_LMCOORDS_BITS;
	}

	num_vbos = 0;
	*vbo_total_size = 0;
	numUnmappedSurfaces = numSurfaces;
	for( i = 0; i < numSurfaces; i++ ) {
		mesh_vbo_t *vbo;
		shader_t *shader;
		int fcount;
		int vcount, ecount;
		vattribmask_t vattribs;
		unsigned last_merged = i;

		if( numUnmappedSurfaces == 0 ) {
			// done
			break;
		}

		// ignore faces already merged
		if( surfmap[i] ) {
			continue;
		}

		surf = surfaces[i];
		shader = surf->shader;
		longrow  = ( int * )( visdata + i * rowbytes );
		arearow = areadata + i * areabytes;

		fcount = 1;
		vcount = surf->mesh.numVerts;
		ecount = surf->mesh.numElems;

		// portal or foliage surfaces can not be batched
		if( !( shader->flags & ( SHADER_PORTAL_CAPTURE | SHADER_PORTAL_CAPTURE2 ) ) && !surf->numInstances ) {
			// scan remaining face checking whether we merge them with the current one
			for( j = i + 1; j < numSurfaces; j++ ) {
				surf2 = surfaces[j];

				// already merged
				if( surf2->drawSurf ) {
					continue;
				}

				// the following checks ensure the two faces are compatible can can be merged
				// into a single vertex buffer object
				if( surf2->shader != surf->shader || surf2->superLightStyle != surf->superLightStyle ) {
					continue;
				}
				if( surf2->fog != surf->fog ) {
					continue;
				}
				if( vcount + surf2->mesh.numVerts >= USHRT_MAX ) {
					continue;
				}
				if( surf2->numInstances != 0 ) {
					continue;
				}

				// unvised maps and submodels submodel can simply skip PVS checks
				if( !visdata ) {
					goto merge;
				}

				// only merge faces that reside in same map areas
				if( areabytes > 0 ) {
					// if areabits aren't equal, faces have different area visibility
					if( memcmp( arearow, areadata + j * areabytes, areabytes ) ) {
						continue;
					}
				}

				// if two faces potentially see same things, we can merge them
				longrow2 = ( int * )( visdata + j * rowbytes );
				for( k = 0; k < rowlongs && !( longrow[k] & longrow2[k] ); k++ ) ;

				if( k != rowlongs ) {
					// merge visibility sets
					for( k = 0; k < rowlongs; k++ )
						longrow[k] |= longrow2[k];
merge:
					fcount++;
					vcount += surf2->mesh.numVerts;
					ecount += surf2->mesh.numElems;
					surfmap[j] = surf;
					last_merged = j;
				}
			}
		}

		// create vertex buffer object for this face then upload data
		vattribs = shader->vattribs | surf->superLightStyle->vattribs | VATTRIB_NORMAL_BIT;
		if( surf->numInstances ) {
			vattribs |= VATTRIB_INSTANCES_BITS;
		}

		// create temp VBO to hold pre-batched info
		if( numTempVBOs == maxTempVBOs ) {
			maxTempVBOs += 1024;
			tempVBOs = (mesh_vbo_s *)Q_realloc( tempVBOs, maxTempVBOs * sizeof( *tempVBOs ) );
		}

		vbo = &tempVBOs[numTempVBOs++];
		vbo->numVerts = vcount;
		vbo->numElems = ecount;
		vbo->vertexAttribs = vattribs;
		if( fcount == 1 ) {
			// non-mergable
			vbo->index = numTempVBOs;
		}

		// allocate a drawsurf
		drawSurf = &loadbmodel->drawSurfaces[loadbmodel->numDrawSurfaces++];
		drawSurf->type = ST_BSP;
		drawSurf->superLightStyle = surf->superLightStyle;
		drawSurf->instances = surf->instances;
		drawSurf->numInstances = surf->numInstances;
		drawSurf->fog = surf->fog;
		drawSurf->shader = surf->shader;
		drawSurf->numLightmaps = 0;

		// upload vertex and elements data for face itself
		surf->drawSurf = loadbmodel->numDrawSurfaces;
		surf->firstDrawSurfVert = 0;
		surf->firstDrawSurfElem = 0;

		vcount = surf->mesh.numVerts;
		ecount = surf->mesh.numElems;
		numUnmappedSurfaces--;

		// count lightmaps
		for( j = 0; j < MAX_LIGHTMAPS; j++ ) {
			if( surf->superLightStyle->lightmapStyles[j] == 255 )
				break;
			drawSurf->numLightmaps++;
		}

		// now if there are any merged faces upload them to the same VBO
		if( fcount > 1 ) {
			for( j = i + 1; j <= last_merged; j++ ) {
				if( surfmap[j] != surf ) {
					continue;
				}

				surf2 = surfaces[j];
				surf2->drawSurf = loadbmodel->numDrawSurfaces;
				surf2->firstDrawSurfVert = vcount;
				surf2->firstDrawSurfElem = ecount;

				vcount += surf2->mesh.numVerts;
				ecount += surf2->mesh.numElems;
				numUnmappedSurfaces--;
			}
		}

		drawSurf->numVerts = vcount;
		drawSurf->numElems = ecount;

		*vbo_total_size += vbo->arrayBufferSize + vbo->elemBufferSize;
	}

	assert( numUnmappedSurfaces == 0 );

	// merge vertex buffer objects with identical vertex attribs
	numUnmergedVBOs = numTempVBOs;
	for( i = 0; i < numTempVBOs; i++ ) {
		mesh_vbo_t *vbo = &tempVBOs[i];

		if( !numUnmergedVBOs ) {
			break;
		}

		if( vbo->index == 0 ) {
			for( j = i + 1; j < numTempVBOs; j++ ) {
				mesh_vbo_t *vbo2 = &tempVBOs[j];

				if( vbo2->index != 0 ) {
					// already merged
					continue;
				}
				if( vbo2->vertexAttribs != vbo->vertexAttribs ) {
					continue;
				}
				if( vbo->numVerts + vbo2->numVerts >= USHRT_MAX ) {
					continue;
				}

				drawSurf = &loadbmodel->drawSurfaces[startDrawSurface + j];
				drawSurf->firstVboVert = vbo->numVerts;
				drawSurf->firstVboElem = vbo->numElems;

				vbo->numVerts += vbo2->numVerts;
				vbo->numElems += vbo2->numElems;

				vbo2->index = i + 1;
				numUnmergedVBOs--;
			}

			vbo->index = i + 1;
		}

		if( vbo->index == i + 1 ) {
			numUnmergedVBOs--;
		}
	}

	assert( numUnmergedVBOs == 0 );

	// create real VBOs and assign owner pointers
	numUnmergedVBOs = numTempVBOs;
	for( i = 0; i < numTempVBOs; i++ ) {
		mesh_vbo_t *vbo = &tempVBOs[i];

		if( !numUnmergedVBOs ) {
			break;
		}

		if( vbo->owner != NULL ) {
			// already assigned to a real VBO
			continue;
		}
		if( vbo->index != i + 1 ) {
			// not owning self, meaning it's been merged to another VBO
			continue;
		}

		drawSurf = &loadbmodel->drawSurfaces[startDrawSurface + i];

		// don't use half-floats for XYZ due to precision issues
		vbo->owner = R_CreateMeshVBO( drawSurf, vbo->numVerts, vbo->numElems, drawSurf->numInstances,
									  vbo->vertexAttribs, VBO_TAG_WORLD, vbo->vertexAttribs & ~floatVattribs );
		drawSurf->vbo = (mesh_vbo_s *)vbo->owner;

		if( drawSurf->numInstances == 0 ) {
			for( j = i + 1; j < numTempVBOs; j++ ) {
				mesh_vbo_t *vbo2 = &tempVBOs[j];

				if( vbo2->index != i + 1 ) {
					continue;
				}

				vbo2->owner = vbo->owner;
				drawSurf = &loadbmodel->drawSurfaces[startDrawSurface + j];
				drawSurf->vbo = (mesh_vbo_s *)vbo->owner;
				numUnmergedVBOs--;
			}
		}

		num_vbos++;
		numUnmergedVBOs--;
	}

	assert( numUnmergedVBOs == 0 );

	// upload data to merged VBO's and assign offsets to drawSurfs
	for( i = 0; i < numSurfaces; i++ ) {
		mesh_vbo_t *vbo;
		const mesh_t *mesh;
		int vertsOffset, elemsOffset;

		surf = surfaces[i];

		if( !surf->drawSurf ) {
			continue;
		}

		drawSurf = &loadbmodel->drawSurfaces[surf->drawSurf - 1];
		mesh = &surf->mesh;
		vbo = drawSurf->vbo;

		vertsOffset = drawSurf->firstVboVert + surf->firstDrawSurfVert;
		elemsOffset = drawSurf->firstVboElem + surf->firstDrawSurfElem;

		R_UploadVBOVertexData( vbo, vertsOffset, vbo->vertexAttribs, mesh );
		R_UploadVBOElemData( vbo, vertsOffset, elemsOffset, mesh );
		R_UploadVBOInstancesData( vbo, 0, surf->numInstances, surf->instances );
	}

	bm->numModelDrawSurfaces = loadbmodel->numDrawSurfaces - bm->firstModelDrawSurface;

	Q_free( tempVBOs );
	Q_free( surfmap );
	Q_free( surfaces );

	if( visdata ) {
		Q_free( visdata );
	}
	if( areadata ) {
		Q_free( areadata );
	}

	return num_vbos;
}

/*
* Mod_CreateVertexBufferObjects
*/
void Mod_CreateVertexBufferObjects( model_t *mod ) {
	unsigned int i;
	unsigned int vbos = 0, total = 0;
	size_t size = 0, total_size = 0;
	mbrushmodel_t *loadbmodel = ( ( mbrushmodel_t * )mod->extradata );

	// free all VBO's allocated for previous world map so
	// we won't end up with both maps residing in video memory
	// until R_FreeUnusedVBOs call
	if( r_prevworldmodel && r_prevworldmodel->registrationSequence != rsh.registrationSequence ) {
		R_FreeVBOsByTag( VBO_TAG_WORLD );
	}

	// allocate memory for drawsurfs
	loadbmodel->numDrawSurfaces = 0;
	loadbmodel->drawSurfaces = (drawSurfaceBSP_t *)Q_malloc( sizeof( *loadbmodel->drawSurfaces ) * loadbmodel->numsurfaces );

	for( i = 0; i < loadbmodel->numsubmodels; i++ ) {
		vbos = Mod_CreateSubmodelBufferObjects( mod, i, &size );
		total += vbos;
		total_size += size;
	}

	for( i = 0; i < loadbmodel->numsubmodels; i++ ) {
		Mod_SortModelSurfaces( mod, i );
	}

	if( total ) {
		VBO_Printf( "Created %i VBOs, totalling %.1f MiB of memory\n", total, ( total_size + 1048574 ) / 1048576.0f );
	}
}

/*
* Mod_FinalizeBrushModel
*/
static void Mod_FinalizeBrushModel( model_t *model ) {
	Mod_FinishFaces( model );

	Mod_CreateVisLeafs( model );

	Mod_CreateVertexBufferObjects( model );

	Mod_SetupSubmodels( model );
}

/*
* Mod_TouchBrushModel
*/
static void Mod_TouchBrushModel( model_t *model ) {
	unsigned int i;
	unsigned int modnum;
	mbrushmodel_t *loadbmodel;

	assert( model );

	loadbmodel = ( ( mbrushmodel_t * )model->extradata );
	assert( loadbmodel );

	for( modnum = 0; modnum < loadbmodel->numsubmodels; modnum++ ) {
		loadbmodel->inlines[modnum].registrationSequence = rsh.registrationSequence;
	}

	// touch all shaders and vertex buffer objects for this bmodel

	for( i = 0; i < loadbmodel->numDrawSurfaces; i++ ) {
		drawSurfaceBSP_t *drawSurf = &loadbmodel->drawSurfaces[i];
		R_TouchShader( drawSurf->shader );
		R_TouchMeshVBO( drawSurf->vbo );
	}

	for( i = 0; i < loadbmodel->numfogs; i++ ) {
		if( loadbmodel->fogs[i].shader ) {
			R_TouchShader( loadbmodel->fogs[i].shader );
		}
	}

	R_TouchLightmapImages( model );
}

//===============================================================================

/*
* R_InitModels
*/
void R_InitModels( void ) {
	memset( mod_novis, 0xff, sizeof( mod_novis ) );
	mod_isworldmodel = false;
	r_prevworldmodel = NULL;
	mod_mapConfigs = (decltype( mod_mapConfigs ))Q_malloc( sizeof( *mod_mapConfigs ) * MAX_MOD_KNOWN );
}

/*
* Mod_Free
*/
static void Mod_Free( model_t *model ) {
	memset( model, 0, sizeof( *model ) );
	model->type = mod_free;
}

/*
* R_FreeUnusedModels
*/
void R_FreeUnusedModels( void ) {
	int i;
	model_t *mod;

	for( i = 0, mod = mod_known; i < mod_numknown; i++, mod++ ) {
		if( !mod->name ) {
			continue;
		}
		if( mod->registrationSequence == rsh.registrationSequence ) {
			// we need this model
			continue;
		}

		Mod_Free( mod );
	}

	// check whether the world model has been freed
	if( rsh.worldModel && rsh.worldModel->type == mod_free ) {
		rsh.worldModel = NULL;
		rsh.worldBrushModel = NULL;
	}
}

/*
* R_ShutdownModels
*/
void R_ShutdownModels( void ) {
	for( int i = 0; i < mod_numknown; i++ ) {
		if( mod_known[i].name ) {
			Mod_Free( &mod_known[i] );
		}
	}

	rsh.worldModel = NULL;
	rsh.worldBrushModel = NULL;

	mod_numknown = 0;
	memset( mod_known, 0, sizeof( mod_known ) );
}

/*
* Mod_StripLODSuffix
*/
void Mod_StripLODSuffix( char *name ) {
	size_t len;

	len = strlen( name );
	if( len <= 2 ) {
		return;
	}
	if( name[len - 2] != '_' ) {
		return;
	}

	if( name[len - 1] >= '0' && name[len - 1] <= '0' + MOD_MAX_LODS ) {
		name[len - 2] = 0;
	}
}

/*
* Mod_FindSlot
*/
static model_t *Mod_FindSlot( const char *name ) {
	int i;
	model_t *mod, *best;

	//
	// search the currently loaded models
	//
	for( i = 0, mod = mod_known, best = NULL; i < mod_numknown; i++, mod++ ) {
		if( mod->type == mod_free ) {
			if( !best ) {
				best = mod;
			}
			continue;
		}
		if( !Q_stricmp( mod->name, name ) ) {
			return mod;
		}
	}

	//
	// return best candidate
	//
	if( best ) {
		return best;
	}

	//
	// find a free model slot spot
	//
	if( mod_numknown == MAX_MOD_KNOWN ) {
		Com_Error( ERR_DROP, "mod_numknown == MAX_MOD_KNOWN" );
	}
	return &mod_known[mod_numknown++];
}

/*
* Mod_Handle
*/
unsigned int Mod_Handle( const model_t *mod ) {
	return mod - mod_known;
}

/*
* Mod_ForHandle
*/
model_t *Mod_ForHandle( unsigned int elem ) {
	return mod_known + elem;
}

/*
* Mod_ForName
*
* Loads in a model for the given name
*/
model_t *Mod_ForName( const char *name, bool crash ) {
	int i;
	model_t *mod, *lod;
	unsigned *buf;
	char shortname[MAX_QPATH], lodname[MAX_QPATH];
	const char *extension;
	const modelFormatDescr_t *descr;
	bspFormatDesc_t *bspFormat = NULL;

	if( !name[0] ) {
		Com_Error( ERR_DROP, "Mod_ForName: NULL name" );
	}

	//
	// inline models are grabbed only from worldmodel
	//
	if( name[0] == '*' ) {
		int modnum = atoi( name + 1 );
		if( modnum < 1 || !rsh.worldModel || (unsigned)modnum >= rsh.worldBrushModel->numsubmodels ) {
			Com_Error( ERR_DROP, "bad inline model number" );
		}
		return &rsh.worldBrushModel->inlines[modnum];
	}

	Q_strncpyz( shortname, name, sizeof( shortname ) );
	COM_StripExtension( shortname );
	extension = &name[strlen( shortname ) + 1];

	mod = Mod_FindSlot( name );
	if( mod->type == mod_bad ) {
		return NULL;
	}
	if( mod->type != mod_free ) {
		return mod;
	}

	//
	// load the file
	//
	modfilelen = R_LoadFile( name, (void **)&buf );
	if( !buf && crash ) {
		Com_Error( ERR_DROP, "Mod_NumForName: %s not found", name );
	}

	mod->type = mod_bad;
	mod->name = (char *)Q_malloc( strlen( name ) + 1 );
	strcpy( mod->name, name );

	// return the NULL model
	if( !buf ) {
		return NULL;
	}

	// call the apropriate loader
	descr = Q_FindFormatDescriptor( mod_supportedformats, ( const uint8_t * )buf, (const bspFormatDesc_t **)&bspFormat );
	if( !descr ) {
		Com_DPrintf( S_COLOR_YELLOW "Mod_NumForName: unknown fileid for %s", mod->name );
		return NULL;
	}

	if( mod_isworldmodel ) {
		// we only init map config when loading the map from disk
		R_InitMapConfig( name );
	}

	descr->loader( mod, NULL, buf, bspFormat );
	R_FreeFile( buf );

	if( mod->type == mod_bad ) {
		return NULL;
	}

	if( mod_isworldmodel ) {
		// we only init map config when loading the map from disk
		R_FinishMapConfig( mod );
	}

	// do some common things
	if( mod->type == mod_brush ) {
		Mod_FinalizeBrushModel( mod );
		mod->touch = &Mod_TouchBrushModel;
	}

	if( !descr->maxLods ) {
		return mod;
	}

	//
	// load level-of-detail models
	//
	mod->lodnum = 0;
	mod->numlods = 0;
	for( i = 0; i < descr->maxLods; i++ ) {
		Q_snprintfz( lodname, sizeof( lodname ), "%s_%i.%s", shortname, i + 1, extension );
		R_LoadFile( lodname, (void **)&buf );
		if( !buf || strncmp( (const char *)buf, descr->header, descr->headerLen ) ) {
			break;
		}

		lod = mod->lods[i] = Mod_FindSlot( lodname );
		if( lod->name && !strcmp( lod->name, lodname ) ) {
			continue;
		}

		lod->type = mod_bad;
		lod->lodnum = i + 1;
		lod->name = (char *)Q_malloc( strlen( lodname ) + 1 );
		strcpy( lod->name, lodname );

		mod_numknown++;

		descr->loader( lod, mod, buf, bspFormat );
		R_FreeFile( buf );

		mod->numlods++;
	}

	return mod;
}

/*
* R_TouchModel
*/
static void R_TouchModel( model_t *mod ) {
	int i;
	model_t *lod;

	if( mod->registrationSequence == rsh.registrationSequence ) {
		return;
	}

	// touching a model precaches all images and possibly other assets
	mod->registrationSequence = rsh.registrationSequence;
	if( mod->touch ) {
		mod->touch( mod );
	}

	// handle Level Of Details
	for( i = 0; i < mod->numlods; i++ ) {
		lod = mod->lods[i];
		lod->registrationSequence = rsh.registrationSequence;
		if( lod->touch ) {
			lod->touch( lod );
		}
	}
}

//=============================================================================

/*
* R_InitMapConfig
*
* Clears map config before loading the map from disk. NOT called when the map
* is reloaded from model cache.
*/
static void R_InitMapConfig( const char *model ) {
	memset( &mapConfig, 0, sizeof( mapConfig ) );

	mapConfig.lightmapsPacking = false;
	mapConfig.lightmapArrays = false;
	mapConfig.maxLightmapSize = 0;
	mapConfig.deluxeMaps = false;
	mapConfig.deluxeMappingEnabled = false;
	mapConfig.forceClear = false;
	mapConfig.forceWorldOutlines = false;
	mapConfig.averageLightingIntensity = 1;

	VectorClear( mapConfig.ambient );
	VectorClear( mapConfig.outlineColor );

	if( r_lighting_packlightmaps->integer ) {
		char lightmapsPath[MAX_QPATH], *p;

		mapConfig.lightmapsPacking = true;

		Q_strncpyz( lightmapsPath, model, sizeof( lightmapsPath ) );
		p = strrchr( lightmapsPath, '.' );
		if( p ) {
			*p = 0;
			Q_strncatz( lightmapsPath, "/lm_0000.tga", sizeof( lightmapsPath ) );
			if( FS_FOpenFile( lightmapsPath, NULL, FS_READ ) != -1 ) {
				Com_DPrintf( S_COLOR_YELLOW "External lightmap stage: lightmaps packing is disabled\n" );
				mapConfig.lightmapsPacking = false;
			}
		}
	}
}

/*
* R_FinishMapConfig
*
* Called after loading the map from disk.
*/
static void R_FinishMapConfig( const model_t *mod ) {
	// ambient lighting
	if( r_fullbright->integer ) {
		VectorSet( mapConfig.ambient, 1, 1, 1 );
		mapConfig.averageLightingIntensity = 1;
	} else {
		ColorNormalize( mapConfig.ambient,  mapConfig.ambient );
	}

	// A dirty hack for fixing a crooked sky appearance
	if( const char *name = mod->name ) {
		for( const char *map : { "maps/wdm1.bsp", "maps/wdm15.bsp" } ) {
			if( !Q_stricmp( name, map ) ) {
				mapConfig.skipSky = true;
				break;
			}
		}
	}

	mod_mapConfigs[mod - mod_known] = mapConfig;
}

//=============================================================================

/*
* R_RegisterWorldModel
*
* Specifies the model that will be used as the world
*/
void R_RegisterWorldModel( const char *model ) {
	r_prevworldmodel = rsh.worldModel;
	rsh.worldModel = NULL;
	rsh.worldBrushModel = NULL;
	rsh.worldModelSequence++;

	mod_isworldmodel = true;

	rsh.worldModel = Mod_ForName( model, true );

	mod_isworldmodel = false;

	if( !rsh.worldModel ) {
		return;
	}

	// FIXME: this is ugly...
	mapConfig = mod_mapConfigs[rsh.worldModel - mod_known];

	R_TouchModel( rsh.worldModel );
	rsh.worldBrushModel = ( mbrushmodel_t * )rsh.worldModel->extradata;
}

/*
* R_RegisterModel
*/
struct model_s *R_RegisterModel( const char *name ) {
	model_t *mod;

	mod = Mod_ForName( name, false );
	if( mod ) {
		R_TouchModel( mod );
	}
	return mod;
}

/*
* R_ModelBounds
*/
void R_ModelBounds( const model_t *model, vec3_t mins, vec3_t maxs ) {
	if( model ) {
		VectorCopy( model->mins, mins );
		VectorCopy( model->maxs, maxs );
	} else if( rsh.worldModel ) {
		VectorCopy( rsh.worldModel->mins, mins );
		VectorCopy( rsh.worldModel->maxs, maxs );
	}
}

/*
* R_ModelFrameBounds
*/
void R_ModelFrameBounds( const struct model_s *model, int frame, vec3_t mins, vec3_t maxs ) {
	if( model ) {
		switch( model->type ) {
			case mod_alias:
				R_AliasModelFrameBounds( model, frame, mins, maxs );
				break;
			case mod_skeletal:
				R_SkeletalModelFrameBounds( model, frame, mins, maxs );
				break;
			default:
				break;
		}
	}
}

static vec4_t *r_modelTransformBuf;
static size_t r_modelTransformBufSize;

/*
* R_GetTransformBufferForMesh
*/
void R_GetTransformBufferForMesh( mesh_t *mesh, bool positions, bool normals, bool sVectors ) {
	size_t bufSize = 0;
	int numVerts = mesh->numVerts;
	vec4_t *bufPtr;

	assert( numVerts );

	if( !numVerts || ( !positions && !normals && !sVectors ) ) {
		return;
	}

	if( positions ) {
		bufSize += numVerts;
	}
	if( normals ) {
		bufSize += numVerts;
	}
	if( sVectors ) {
		bufSize += numVerts;
	}
	bufSize *= sizeof( vec4_t );
	if( bufSize > r_modelTransformBufSize ) {
		r_modelTransformBufSize = bufSize;
		if( r_modelTransformBuf ) {
			Q_free( r_modelTransformBuf );
		}
		r_modelTransformBuf = (vec4_t *)Q_malloc( bufSize );
	}

	bufPtr = r_modelTransformBuf;
	if( positions ) {
		mesh->xyzArray = bufPtr;
		bufPtr += numVerts;
	}
	if( normals ) {
		mesh->normalsArray = bufPtr;
		bufPtr += numVerts;
	}
	if( sVectors ) {
		mesh->sVectorsArray = bufPtr;
	}
}
