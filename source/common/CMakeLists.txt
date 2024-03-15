project(common)

file(GLOB WSWCOMMON_SOURCES
    "base64.cpp"
    "cmdargssplitter.cpp"
    "configvars.cpp"
    "gs_*.cpp"
    "hash.cpp"
    "md5.cpp"
    "q_math.cpp"
    "q_shared.cpp"
    "textstreamwriter.cpp"
    "wswexceptions.cpp"
    "wswfs.cpp"
    "wswsortbyfield.cpp"
    "wswstringview.cpp"
    "userinfo.cpp")

file(GLOB WSWCOMMON_HEADERS
    "base64.h"
    "cmdargs.h"
    "configvars.h"
    "md5.h"
    "gs_*.h"
    "q_*.h"
    "wswexceptions.h"
    "wswsortbyfield.h"
    "wswstringview.h")

add_library(wswcommon ${WSWCOMMON_SOURCES} ${WSWCOMMON_HEADERS})
set_property(TARGET wswcommon PROPERTY POSITION_INDEPENDENT_CODE ON)
qf_set_output_dir(wswcommon "")

if (NOT GAME_MODULES_ONLY)
    include_directories(${ZLIB_INCLUDE_DIR} ${CURL_INCLUDE_DIR})

    file(GLOB EXECOMMON_SOURCES
         "asyncstream.cpp"
         "autoupdate.cpp"
         "bsp.cpp"
         "cm_*.cpp"
         "cmdsystem.cpp"
         "common.cpp"
         "compression.cpp"
         "configstringstorage.cpp"
         "cvar.cpp"
         "files.cpp"
         "glob.cpp"
         "half_float.cpp"
         "library.cpp"
         "net.cpp"
         "net_chan.cpp"
         "maplist.cpp"
         "mmcommon.cpp"
         "msg.cpp"
         "outputmessages.cpp"
         "patch.cpp"
         "q_trie.cpp"
         "snap_demos.cpp"
         "threads.cpp"
         "../server/sv_web.cpp"
         "../server/sv_main.cpp"
         "wswcurl.cpp")

    file(GLOB EXECOMMON_HEADERS
         "../common/bsp.h"
         "outputmessages.h"
         "q_trie.h")

    if (MSVC)
        set_source_files_properties("../common/cm_trace_avx.cpp" PROPERTIES COMPILE_FLAGS "/arch:AVX")
    else()
        set_source_files_properties("../common/cm_trace_sse42.cpp" PROPERTIES COMPILE_FLAGS "-msse4.2")
        set_source_files_properties("../common/cm_trace_avx.cpp" PROPERTIES COMPILE_FLAGS "-mavx")
    endif()

    add_library(execommon ${EXECOMMON_SOURCES} ${EXECOMMON_HEADERS}
            geometry.cpp)
    target_link_libraries(execommon PUBLIC wswcommon)
    target_link_libraries(execommon PRIVATE ${CURL_LIBRARY} ${ZLIB_LIBRARY})
    qf_set_output_dir(execommon "")
endif()
