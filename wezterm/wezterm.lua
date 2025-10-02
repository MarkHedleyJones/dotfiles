local wezterm = require("wezterm")
local config = wezterm.config_builder()

-- Appearance
config.enable_scroll_bar = false

-- Font configuration
config.font = wezterm.font("Terminus (TTF)")
config.font_size = 12.0

-- Tokyo Night Storm colour scheme
config.color_scheme = "tokyonight-storm"

-- Window padding
config.window_padding = {
	left = 4,
	right = 4,
	top = 4,
	bottom = 4,
}

-- Tab bar
config.hide_tab_bar_if_only_one_tab = true
config.use_fancy_tab_bar = false

-- Disable audible bell
config.audible_bell = "Disabled"

-- Scrollback
config.scrollback_lines = 10000

return config
