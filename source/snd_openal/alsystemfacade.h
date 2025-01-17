#include "../gameshared/q_math.h"
#include "../client/snd_public.h"
#include "../qcommon/wswstaticvector.h"

#include "backend.h"
#include "snd_local.h"

namespace wsw::snd {

class ALSoundSystem : public SoundSystem {
public:
	struct ThreadProcArg {
		qbufPipe_s *pipe;
		ALSoundSystem *instance;
	};

	[[nodiscard]]
	static auto tryCreate( client_state_s *client, bool verbose ) -> ALSoundSystem *;

	ALSoundSystem( client_state_s *client, qbufPipe_s *pipe, qthread_s *thread, bool verbose );
	~ALSoundSystem() override;

	void deleteSelf( bool verbose ) override;

	void postInit() override;

	void beginRegistration() override;
	void endRegistration() override;

	void stopAllSounds( unsigned flags ) override;

	void clear() override;

	void updateListener( const float *origin, const float *velocity, const mat3_t axis ) override;

	void activate( bool isActive ) override;

	void processFrameUpdates() override;

	void setEntitySpatialParams( int entNum, const float *origin, const float *velocity ) override;

	[[nodiscard]]
	auto registerSound( const char *name ) -> sfx_s * override;

	void startFixedSound( sfx_s *sfx, const float *origin, int channel, float volume, float attenuation ) override;
	void startRelativeSound( sfx_s *sfx, int entNum, int channel, float volume, float attenuation ) override;
	void startGlobalSound( sfx_s *sfx, int channel, float volume ) override;
	void startLocalSound( const char *name, float volume ) override;
	void startLocalSound( sfx_s *sfx, float volume ) override;

	void addLoopSound( sfx_s *sfx, int entNum, uintptr_t identifyingToken, float volume, float attenuation ) override;

	void startBackgroundTrack( const char *intro, const char *loop, int mode ) override;
	void stopBackgroundTrack() override;
	void prevBackgroundTrack() override;
	void nextBackgroundTrack() override;
	void pauseBackgroundTrack() override;
private:
	void flushEntitySpatialParams();

	wsw::String m_tmpPathBuffer1;
	wsw::String m_tmpPathBuffer2;

	Backend::EntitySpatialParamsBatch m_spatialParamsBatch;

	qthread_s *m_thread { nullptr };
	qbufPipe_s *m_pipe { nullptr };
	bool m_useVerboseShutdown { false };

	Backend m_backend;
};

}