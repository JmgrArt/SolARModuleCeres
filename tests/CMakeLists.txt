cmake_minimum_required(VERSION 3.7.2)

##################################################
project("SolARTestTriangulation")
set (SOURCES main.cpp)
##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()

# define the list of files to copy to build directory
if (UNIX)
        set (LIBPREFIX "lib")
        set (LIBEXTENSION "so")
endif(UNIX)
if (WIN32)
    set (CERES_EXT "")
    if(CMAKE_BUILD_TYPE STREQUAL "Debug" )
        set (CERES_EXT "-debug")
    endif(DEBUG_CONFIGURATIONS)
        set (LIBPREFIX "")
        set (LIBEXTENSION "dll")
endif(WIN32)
set (BUILDCONFIG $<$<CONFIG:Debug>:debug>$<$<NOT:$<CONFIG:Debug>>:release>)

set(FILES_TO_COPY

        $ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_filesystem.${LIBEXTENSION}
        $ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_system.${LIBEXTENSION}
        $ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_timer.${LIBEXTENSION}
        $ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_log.${LIBEXTENSION}
        $ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_chrono.${LIBEXTENSION}
        $ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_thread.${LIBEXTENSION}
        $ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_date_time.${LIBEXTENSION}
        $ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_fiber.${LIBEXTENSION}
        $ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_context.${LIBEXTENSION}

        $ENV{BCOMDEVROOT}/thirdParties/ceres/1.14.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}ceres${CERES_EXT}.${LIBEXTENSION}
        $ENV{BCOMDEVROOT}/thirdParties/gflags/2.2.1/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}gflags.${LIBEXTENSION}
        $ENV{BCOMDEVROOT}/thirdParties/glog/0.3.5/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}glog.${LIBEXTENSION}
        $ENV{BCOMDEVROOT}/thirdParties/freeglut/3.0.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}freeglut.${LIBEXTENSION}

	$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/0.4.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}SolARFramework.${LIBEXTENSION}
        $ENV{BCOMDEVROOT}/thirdParties/xpcf/2.1.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}xpcf.${LIBEXTENSION}
	)
# define targets (library, install and uninstall)
defineTargets("executable" "${FILES_TO_COPY}")

