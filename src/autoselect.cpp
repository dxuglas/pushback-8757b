#include "main.h"
#include "liblvgl/lvgl.h"
#include "utils/config.h"

// Autonomous routine names
const char* auton_names[] = {
    "Left Loader",
    "Left Center",
    "Right Loader",
    "Right Center"
};

// LVGL objects
static lv_obj_t* alliance_toggle;
static lv_obj_t* auton_buttons[4];
static lv_obj_t* status_label;

// Alliance toggle callback
static void alliance_toggle_cb(lv_event_t* e) {
    lv_obj_t* toggle = (lv_obj_t*)lv_event_get_target(e);
    bool is_checked = lv_obj_has_state(toggle, LV_STATE_CHECKED);
    
    // Set alliance: checked = red (1), unchecked = blue (-1)
    alliance = is_checked ? 1 : -1;
    
    // Update status label
    lv_label_set_text_fmt(status_label, "%s | %s", 
                          alliance == 1 ? "RED" : "BLUE", 
                          auton_names[selected_auton]);
    
    // Update toggle colors
    if (alliance == 1) {
        lv_obj_set_style_bg_color(alliance_toggle, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_CHECKED);
    } else {
        lv_obj_set_style_bg_color(alliance_toggle, lv_color_hex(0x0000FF), LV_PART_INDICATOR | LV_STATE_CHECKED);
    }
}

// Autonomous button callback
static void auton_button_cb(lv_event_t* e) {
    lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
    int* btn_id = (int*)lv_event_get_user_data(e);
    
    // Deselect all buttons
    for (int i = 0; i < 4; i++) {
        lv_obj_remove_state(auton_buttons[i], LV_STATE_CHECKED);
    }
    
    // Select clicked button
    lv_obj_add_state(btn, LV_STATE_CHECKED);
    selected_auton = *btn_id;
    
    // Update status label
    lv_label_set_text_fmt(status_label, "%s | %s", 
                          alliance == 1 ? "RED" : "BLUE", 
                          auton_names[selected_auton]);
}

void initialize_autonomous_selector() {
    // Create main container
    lv_obj_t* screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);
    
    // Title label - smaller and at top
    lv_obj_t* title = lv_label_create(screen);
    lv_label_set_text(title, "Autonomous Selector");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);
    
    // Alliance section - compact layout
    lv_obj_t* alliance_label = lv_label_create(screen);
    lv_label_set_text(alliance_label, "Alliance:");
    lv_obj_set_style_text_font(alliance_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(alliance_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(alliance_label, LV_ALIGN_TOP_LEFT, 10, 30);
    
    // Alliance toggle switch
    alliance_toggle = lv_switch_create(screen);
    lv_obj_set_size(alliance_toggle, 50, 25);
    lv_obj_align(alliance_toggle, LV_ALIGN_TOP_LEFT, 90, 28);
    lv_obj_add_state(alliance_toggle, LV_STATE_CHECKED); // Start as red (1)
    lv_obj_set_style_bg_color(alliance_toggle, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(alliance_toggle, alliance_toggle_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    // Alliance label (Red/Blue indicator)
    lv_obj_t* alliance_text = lv_label_create(screen);
    lv_label_set_text(alliance_text, "RED / BLUE");
    lv_obj_set_style_text_font(alliance_text, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(alliance_text, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(alliance_text, LV_ALIGN_TOP_LEFT, 150, 32);
    
    // Autonomous routines section - 2x2 grid layout
    lv_obj_t* auton_label = lv_label_create(screen);
    lv_label_set_text(auton_label, "Select Routine:");
    lv_obj_set_style_text_font(auton_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(auton_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(auton_label, LV_ALIGN_TOP_LEFT, 10, 65);
    
    // Create autonomous routine buttons in 2x2 grid
    static int button_ids[4] = {0, 1, 2, 3};
    int button_width = 220;
    int button_height = 50;
    int x_offset[4] = {20, 250, 20, 250};  // Left, Right, Left, Right
    int y_offset[4] = {95, 95, 155, 155};   // Top, Top, Bottom, Bottom
    
    for (int i = 0; i < 4; i++) {
        auton_buttons[i] = lv_button_create(screen);
        lv_obj_set_size(auton_buttons[i], button_width, button_height);
        lv_obj_set_pos(auton_buttons[i], x_offset[i], y_offset[i]);
        
        // Add checkable flag
        lv_obj_add_flag(auton_buttons[i], LV_OBJ_FLAG_CHECKABLE);
        
        // Style for checked state
        lv_obj_set_style_bg_color(auton_buttons[i], lv_color_hex(0x00AA00), LV_STATE_CHECKED);
        
        // Button label
        lv_obj_t* btn_label = lv_label_create(auton_buttons[i]);
        lv_label_set_text(btn_label, auton_names[i]);
        lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_14, 0);
        lv_obj_center(btn_label);
        
        // Add callback
        lv_obj_add_event_cb(auton_buttons[i], auton_button_cb, LV_EVENT_CLICKED, &button_ids[i]);
    }
    
    // Select first button by default
    lv_obj_add_state(auton_buttons[0], LV_STATE_CHECKED);
    
    // Status label at bottom - compact
    status_label = lv_label_create(screen);
    lv_label_set_text_fmt(status_label, "RED | %s", auton_names[0]);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xFFFF00), 0);
    lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID, 0, -5);
}
