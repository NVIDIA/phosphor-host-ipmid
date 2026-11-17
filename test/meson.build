gtest = dependency('gtest', main: true, disabler: true, required: false)
gmock = dependency('gmock', disabler: true, required: false)
if not gtest.found() or not gmock.found()
    gtest_opts = import('cmake').subproject_options()
    gtest_opts.add_cmake_defines({'CMAKE_CXX_FLAGS': '-Wno-pedantic'})
    gtest_proj = import('cmake').subproject(
        'googletest',
        options: gtest_opts,
        required: false,
    )
    if gtest_proj.found()
        gtest = declare_dependency(
            dependencies: [
                dependency('threads'),
                gtest_proj.dependency('gtest'),
                gtest_proj.dependency('gtest_main'),
            ],
        )
        gmock = gtest_proj.dependency('gmock')
    else
        assert(not get_option('tests').enabled(), 'Googletest is required')
    endif
endif

test(
    'dbus-sdr/stable_ids',
    executable(
        'dbus-sdr_stable_ids',
        'dbus-sdr/stable_ids_test.cpp',
        include_directories: root_inc,
        dependencies: gtest,
    ),
)

test(
    'dbus-sdr/sdr_repository',
    executable(
        'dbus-sdr_sdr_repository',
        'dbus-sdr/sdr_repository_test.cpp',
        include_directories: root_inc,
        dependencies: gtest,
    ),
)

test(
    'dbus-sdr/fru_discovery',
    executable(
        'dbus-sdr_fru_discovery',
        'dbus-sdr/fru_discovery_test.cpp',
        include_directories: root_inc,
        dependencies: gtest,
    ),
)

test(
    'dbus-sdr/sdr_build',
    executable(
        'dbus-sdr_sdr_build',
        'dbus-sdr/sdr_build_test.cpp',
        include_directories: root_inc,
        dependencies: gtest,
    ),
)

test(
    'entitymap_json',
    executable(
        'entitymap_json',
        'entitymap_json_unittest.cpp',
        include_directories: root_inc,
        build_by_default: false,
        implicit_include_directories: false,
        dependencies: [
            entity_map_json_dep,
            gmock,
            gtest,
            nlohmann_json_dep,
            sdbusplus_dep,
        ],
    ),
)

# Build/add oemrouter_unittest to test suite
# Issue #3325
# test('oemrouter',
#  executable(
#    'oemrouter',
#    'oemrouter_unittest.cpp',
#    include_directories: root_inc,
#    build_by_default: false,
#    implicit_include_directories: false,
#    dependencies: [gtest, gmock]
#  ))

# Build/add message packing/unpacking unit tests
test(
    'message',
    executable(
        'message',
        'message/pack.cpp',
        'message/payload.cpp',
        'message/unpack.cpp',
        include_directories: root_inc,
        build_by_default: false,
        override_options: ['b_lundef=true'],
        implicit_include_directories: false,
        dependencies: [
            boost,
            crypto,
            gmock,
            gtest,
            libsystemd_dep,
            phosphor_logging_dep,
            sdbusplus_dep,
        ],
    ),
)

# Build/add closesession_unittest to test suite
test(
    'session/closesession',
    executable(
        'session_closesession',
        'session/closesession_unittest.cpp',
        include_directories: root_inc,
        build_by_default: false,
        implicit_include_directories: false,
        dependencies: [gtest, gmock],
    ),
)

# Build/add selseverity_unittest to test suite
test(
    'dbus-sdr/selseverity',
    executable(
        'dbus-sdr_selseverity',
        'dbus-sdr/selseverity_unittest.cpp',
        include_directories: root_inc,
        build_by_default: false,
        implicit_include_directories: false,
        dependencies: [
            ipmid_dep,
            phosphor_dbus_interfaces_dep,
            phosphor_logging_dep,
            sdbusplus_dep,
            gtest,
            gmock,
        ],
    ),
)

# Build/add sensorcommands_unittest to test suite
test(
    'dbus-sdr/sensorcommands',
    executable(
        'dbus-sdr_sensorcommands',
        'dbus-sdr/sensorcommands_unittest.cpp',
        include_directories: root_inc,
        build_by_default: false,
        implicit_include_directories: false,
        dependencies: [sensorutils_dep, gtest, gmock],
    ),
)

test(
    'dbus-sdr/fru_names',
    executable(
        'dbus-sdr_fru_names',
        'dbus-sdr/fru_names_test.cpp',
        include_directories: root_inc,
        dependencies: gtest,
    ),
)

test(
    'dbus-sdr/sdr_review',
    executable(
        'dbus-sdr_sdr_review',
        'dbus-sdr/sdr_review_test.cpp',
        include_directories: root_inc,
        dependencies: gtest,
    ),
)
