idf_component_register(
    SRC_DIRS     src
    #SRCS
    #    "src/platform.c"
    #    "src/vl53l7cx_api.c"
    #    "src/vl53l7cx_buffers.c"
        # plugins staan bij jou (nog) in de component root:
        # "vl53l7cx_plugin_detection_thresholds.c"
        # "vl53l7cx_plugin_motion_indicator.c"
        # "vl53l7cx_plugin_xtalk.c"

    INCLUDE_DIRS
        "include"

    REQUIRES
        driver
        freertos
        log
)
