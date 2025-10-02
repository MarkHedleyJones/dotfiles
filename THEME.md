# GTK Theme Setup

## GTK4 Theme Manual Setup

GTK4 handles themes differently than GTK3. While GTK3 reads `gtk-theme-name` from settings.ini and loads themes from `~/.themes/`, GTK4 requires CSS files to be directly in `~/.config/gtk-4.0/`.

### Manual Setup Steps

If `install.sh` doesn't handle it or you need to fix broken GTK4 theming:

```bash
# Replace THEME_NAME with your theme (e.g., catppuccin-mocha-mauve-standard+default)
THEME_NAME="your-theme-name-here"

ln -sf ~/.themes/${THEME_NAME}/gtk-4.0/gtk.css ~/.config/gtk-4.0/
ln -sf ~/.themes/${THEME_NAME}/gtk-4.0/gtk-dark.css ~/.config/gtk-4.0/
ln -sf ~/.themes/${THEME_NAME}/gtk-4.0/assets ~/.config/gtk-4.0/
```

### What install.sh Does

The install script automatically:
1. Reads your theme name from `~/.config/gtk-3.0/settings.ini`
2. Creates symlinks from `~/.themes/THEME_NAME/gtk-4.0/*` to `~/.config/gtk-4.0/`
3. Links `gtk.css`, `gtk-dark.css`, and `assets/` directory

### Verifying Setup

```bash
ls -la ~/.config/gtk-4.0/
```

Should show symlinks to your theme's GTK4 files, not just `settings.ini`.

### After Changing Themes

1. Update `~/.config/gtk-3.0/settings.ini` with new theme name
2. Run `./install.sh` to update GTK4 symlinks
3. Restart GTK4 applications
