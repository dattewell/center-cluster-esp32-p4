SET(SOURCES screens/ui_BootScreen.c
    screens/ui_MainSpeedo.c
    screens/ui_OtherData.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_bootscreen_png.c
    images/ui_img_gauge_background_png.c
    images/ui_img_gauge_needle_png.c
    images/ui_img_gauge_png.c
    images/ui_img_needle_no_shadow65_png.c
    fonts/ui_font_DIN_1451_Mittelschrift_.c
    fonts/ui_font_DIN_1451__96.c
    fonts/ui_font_DSGE7_Classic_80.c)

add_library(ui ${SOURCES})
