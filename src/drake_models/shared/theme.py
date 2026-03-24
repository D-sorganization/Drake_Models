"""Shared plotting theme configuration."""
from __future__ import annotations

import logging

logger = logging.getLogger(__name__)

try:
    from plot_theme.themes import get_theme, DEFAULT_THEME
    from plot_theme.integration import style_axis as _style_axis
    
    # Export the default theme
    theme = get_theme(DEFAULT_THEME)
    
    def style_axis(ax) -> None:
        """Apply the global shared plot theme to a Matplotlib axis."""
        _style_axis(ax)
        
    logger.debug(f"Loaded shared plot theme: {theme.name}")

except ImportError:
    logger.warning("ud-tools plot_theme not available. Plotting theme will fallback to defaults.")
    
    theme = None
    
    def style_axis(ax) -> None:
        """Fallback empty styling if plot_theme is missing."""
        pass
