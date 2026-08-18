# =================================================================
# PRODUCTION-GRADE ANSI ESCAPE CHARACTER COLOR & FORMATTING LIBRARY
# =================================================================
# This library provides object-oriented text styling for terminals
# using standard ANSI escape sequences. Supports direct method chaining
# and functional custom print syntax wrappers.

import sys

class ColorCode:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    ITALIC = "\033[3m"
    UNDERLINE = "\033[4m"
    BLINK = "\033[5m"
    REVERSE = "\033[7m"
    HIDDEN = "\033[8m"
    STRIKETHROUGH = "\033[9m"
    
    BLACK = "\033[30m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"
    
    BRIGHT_BLACK = "\033[90m"
    BRIGHT_RED = "\033[91m"
    BRIGHT_GREEN = "\033[92m"
    BRIGHT_YELLOW = "\033[93m"
    BRIGHT_BLUE = "\033[94m"
    BRIGHT_MAGENTA = "\033[95m"
    BRIGHT_CYAN = "\033[96m"
    BRIGHT_WHITE = "\033[97m"
    
    BG_BLACK = "\033[40m"
    BG_RED = "\033[41m"
    BG_GREEN = "\033[42m"
    BG_YELLOW = "\033[43m"
    BG_BLUE = "\033[44m"
    BG_MAGENTA = "\033[45m"
    BG_CYAN = "\033[46m"
    BG_WHITE = "\033[47m"

class TextStyle:
    def __init__(self, current_string="", active_codes=None):
        self._string = str(current_string)
        self._codes = list(active_codes) if active_codes else []

    def _apply(self, code):
        new_codes = list(self._codes)
        if code not in new_codes:
            new_codes.append(code)
        return TextStyle(self._string, new_codes)

    @property
    def bold(self): return self._apply(ColorCode.BOLD)
    @property
    def dim(self): return self._apply(ColorCode.DIM)
    @property
    def italic(self): return self._apply(ColorCode.ITALIC)
    @property
    def underline(self): return self._apply(ColorCode.UNDERLINE)
    @property
    def blink(self): return self._apply(ColorCode.BLINK)
    @property
    def strike(self): return self._apply(ColorCode.STRIKETHROUGH)

    @property
    def red(self): return self._apply(ColorCode.RED)
    @property
    def green(self): return self._apply(ColorCode.GREEN)
    @property
    def blue(self): return self._apply(ColorCode.BLUE)
    @property
    def yellow(self): return self._apply(ColorCode.YELLOW)
    @property
    def magenta(self): return self._apply(ColorCode.MAGENTA)
    @property
    def cyan(self): return self._apply(ColorCode.CYAN)
    @property
    def white(self): return self._apply(ColorCode.WHITE)
    @property
    def black(self): return self._apply(ColorCode.BLACK)

    @property
    def bright_red(self): return self._apply(ColorCode.BRIGHT_RED)
    @property
    def bright_green(self): return self._apply(ColorCode.BRIGHT_GREEN)
    @property
    def bright_blue(self): return self._apply(ColorCode.BRIGHT_BLUE)

    @property
    def bg_red(self): return self._apply(ColorCode.BG_RED)
    @property
    def bg_green(self): return self._apply(ColorCode.BG_GREEN)
    @property
    def bg_blue(self): return self._apply(ColorCode.BG_BLUE)

    def load(self, text_content):
        return TextStyle(text_content, self._codes)

    def print(self, *args, sep=" ", end="\n", file=sys.stdout, flush=False):
        combined_text = sep.join(str(arg) for arg in args) if args else self._string
        prefix = "".join(self._codes)
        suffix = ColorCode.RESET if prefix else ""
        print(f"{prefix}{combined_text}{suffix}", sep=sep, end=end, file=file, flush=flush)

    def __str__(self):
        prefix = "".join(self._codes)
        suffix = ColorCode.RESET if prefix else ""
        return f"{prefix}{self._string}{suffix}"

class TerminalColorManager:
    @property
    def red(self): return TextStyle().red
    @property
    def green(self): return TextStyle().green
    @property
    def blue(self): return TextStyle().blue
    @property
    def yellow(self): return TextStyle().yellow
    @property
    def magenta(self): return TextStyle().magenta
    @property
    def cyan(self): return TextStyle().cyan
    @property
    def white(self): return TextStyle().white
    @property
    def black(self): return TextStyle().black

    @property
    def bright_red(self): return TextStyle().bright_red
    @property
    def bright_green(self): return TextStyle().bright_green
    @property
    def bright_blue(self): return TextStyle().bright_blue

    @property
    def bold(self): return TextStyle().bold
    @property
    def dim(self): return TextStyle().dim
    @property
    def italic(self): return TextStyle().italic
    @property
    def underline(self): return TextStyle().underline
    @property
    def blink(self): return TextStyle().blink
    @property
    def strike(self): return TextStyle().strike

color = TerminalColorManager()

import sys
import shutil


class CleanBanner:
    """A minimal, high-density terminal banner generator designed for clean,

    scannable layout headers.
    """

    def __init__(self):
        # Clean, modern box-drawing characters
        self.h = "─"
        self.v = "│"
        self.tl = "┌"
        self.tr = "┐"
        self.bl = "└"
        self.br = "┘"

    def b(self, text: str) -> str:
        """Generates and displays a clean, tightly framed terminal banner.

        Syntax: banner.b("TEXT")
        """
        # Get terminal size dynamically, fallback to 80 characters
        term_width = shutil.get_terminal_size((80, 20)).columns
        max_width = min(70, term_width - 6)

        # Sanitize and truncate the clean string text
        clean_text = str(text).strip().upper()[:max_width]
        content_width = max(len(clean_text), 30)

        # Build clean structural frame borders
        top = self.tl + (self.h * (content_width + 4)) + self.tr
        middle = (
            f"{self.v}  {clean_text.center(content_width)}  {self.v}"
        )
        bottom = self.bl + (self.h * (content_width + 4)) + self.br

        # Assemble the clean banner matrix
        compiled_banner = f"\n{top}\n{middle}\n{bottom}\n"

        # Stream directly to standard output
        sys.stdout.write(compiled_banner)
        sys.stdout.flush()

        return compiled_banner


# Global object instantiation to match requested execution syntax pattern
banner = CleanBanner()

# =================================================================
# EXAMPLE EXECUTION (Matches your exact pattern requirement)
# =================================================================

