SET(SOURCES screens/ui_MainSpeedo.c
    screens/ui_OtherData.c
    screens/ui_BootScreen.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_gauge_background_png.c
    images/ui_img_gauge_needle_png.c
    images/ui_img_bootscreen_png.c
    fonts/ui_font_DIN_1451_Mittelschrift_.c
    fonts/ui_font_DIN_1451__96.c)

add_library(ui ${SOURCES})
