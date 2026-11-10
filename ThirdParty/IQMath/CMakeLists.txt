# 预编译库来自上游 gcc-cm3 目录，使用 Cortex-M3 指令集及软件浮点调用约定。
add_library(iqmath STATIC IMPORTED GLOBAL)
set_target_properties(iqmath PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/gcc-cm3/IQmathLib-cm3.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}"
)
add_library(IQMath::IQMath ALIAS iqmath)
