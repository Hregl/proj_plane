# ============================================================================
#  tools/CMakeLists.txt  →  离线开发/仿真辅助工具
#
#  依据：批次 B1（同一合成几何只写一份）、批次 Qt 仿真交付
#
#  ⚠ 本目录的目标**不进入生产运行路径**，也不被 AircraftPoseSystem 链接。
#    它们只是把源码里的几何/配置生成为可复核的文件。
#
#  ⚠ 源文件显式列举，不用 GLOB —— 与 tests/ 同一约定：工具被误删或改名时
#    应当在配置期报错，而不是静默少构建一个目标。
# ============================================================================

# ----------------------------------------------------------------------------
# 转台实机验收入口（独立可执行，不进入生产装配）
#
# ⚠ 与 gen_synthetic_model 不同，本工具**会**链接 device，并可能发送实机
#   运动帧；但只在显式 `--permit` 且许可通过生产会话的全部判据时才可能。
#   无许可时是只读会话。它不被 AircraftPoseSystem 链接。
# ----------------------------------------------------------------------------
if(TARGET device AND TARGET infrastructure AND TARGET data)
    add_executable(turntable_acceptance turntable_acceptance.cpp)

    set_target_properties(turntable_acceptance PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )

    target_include_directories(turntable_acceptance PRIVATE
        "${PROJECT_SOURCE_DIR}/src")

    # 复用生产设备层与配置加载；不链 Qt、不链 app/application。
    target_link_libraries(turntable_acceptance PRIVATE
        data
        device
        infrastructure
    )

    install(TARGETS turntable_acceptance
            RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

    message(STATUS "[APS] 工具 turntable_acceptance 已生成")
else()
    message(STATUS
        "[APS] 工具 turntable_acceptance：data/device/infrastructure "
        "尚未生成，跳过")
endif()

if(TARGET algorithm AND TARGET data)
    add_executable(gen_synthetic_model gen_synthetic_model.cpp)

    set_target_properties(gen_synthetic_model PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )

    target_include_directories(gen_synthetic_model PRIVATE
        "${PROJECT_SOURCE_DIR}/src")

    # 只依赖 data + algorithm：模型格式由 TargetModelManager 定义，
    # 几何由 data::synthetic 定义。工具不读配置、不碰设备、不链 Qt。
    target_link_libraries(gen_synthetic_model PRIVATE data algorithm)

    install(TARGETS gen_synthetic_model
            RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

    message(STATUS "[APS] 工具 gen_synthetic_model 已生成")
else()
    message(STATUS
        "[APS] 工具 gen_synthetic_model：data/algorithm 尚未生成，跳过")
endif()
