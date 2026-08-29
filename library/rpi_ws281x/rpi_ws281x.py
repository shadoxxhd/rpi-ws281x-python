# Adafruit NeoPixel library port to the rpi_ws281x library.
# Author: Tony DiCola (tony@tonydicola.com), Jeremy Garff (jer@jers.net)
import _rpi_ws281x as ws
import numpy as np
import sys
import atexit


class RGBW(int):
    def __new__(self, r, g=None, b=None, w=None):
        if (g, b, w) == (None, None, None):
            return int.__new__(self, r)
        else:
            if w is None:
                w = 0
            if g is None:
                g = 0
            if b is None:
                b = 0
            return int.__new__(self, (w << 24) | (r << 16) | (g << 8) | b)

    @property
    def r(self):
        return (self >> 16) & 0xff

    @property
    def g(self):
        return (self >> 8) & 0xff

    @property
    def b(self):
        return (self) & 0xff

    @property
    def w(self):
        return (self >> 24) & 0xff

_int_to_RGBW = np.frompyfunc(RGBW,1,1) # creates a ufunc that turns an array of integer colors into an array of RGBW objects

def Color(red, green, blue, white=0):
    """Convert the provided red, green, blue color to a 24-bit color value.
    Each color component should be a value 0-255 where 0 is the lowest intensity
    and 255 is the highest intensity.
    """
    return RGBW(red, green, blue, white)


class PixelStrip:
    def __init__(self, num, pin, freq_hz=800000, dma=10, invert=False,
            brightness=255, channel=0, strip_type=None, gamma=None):
        """Class to represent a SK6812/WS281x LED display.  Num should be the
        number of pixels in the display, and pin should be the GPIO pin connected
        to the display signal line (must be a PWM pin like 18!).  Optional
        parameters are freq, the frequency of the display signal in hertz (default
        800khz), dma, the DMA channel to use (default 10), invert, a boolean
        specifying if the signal line should be inverted (default False), and
        channel, the PWM channel to use (defaults to 0).

        All the methods of a PixelSubStrip are available on PixelStrip
        objects.
        """

        if gamma is None:
            # Support gamma in place of strip_type for back-compat with
            # previous version of forked library
            if type(strip_type) is list and len(strip_type) == 256:
                gamma = strip_type
                strip_type = None
            else:
                gamma = list(range(256))

        if strip_type is None:
            strip_type = ws.WS2811_STRIP_GRB

        # Create ws2811_t structure and fill in parameters.
        self._leds = ws.new_ws2811_t()

        # Initialize the channels to zero
        for channum in range(2):
            chan = ws.ws2811_channel_get(self._leds, channum)
            ws.ws2811_channel_t_count_set(chan, 0)
            ws.ws2811_channel_t_gpionum_set(chan, 0)
            ws.ws2811_channel_t_invert_set(chan, 0)
            ws.ws2811_channel_t_brightness_set(chan, 0)

        # Initialize the channel in use
        self._channel = ws.ws2811_channel_get(self._leds, channel)

        ws.ws2811_channel_t_gamma_set(self._channel, gamma)
        ws.ws2811_channel_t_count_set(self._channel, num)
        ws.ws2811_channel_t_gpionum_set(self._channel, pin)
        ws.ws2811_channel_t_invert_set(self._channel, 0 if not invert else 1)
        ws.ws2811_channel_t_brightness_set(self._channel, brightness)
        ws.ws2811_channel_t_strip_type_set(self._channel, strip_type)

        # Initialize the controller
        ws.ws2811_t_freq_set(self._leds, freq_hz)
        ws.ws2811_t_dmanum_set(self._leds, dma)

        self.size = num

        # Create a PixelSubStrip and delegate these methods to it
        self.main_strip = self.PixelSubStrip(self)
        self.setPixelColor = self.main_strip.setPixelColor
        self.setPixelColorRGB = self.main_strip.setPixelColorRGB
        self.setBrightness = self.main_strip.setBrightness
        self.getBrightness = self.main_strip.getBrightness
        self.getPixelColor = self.main_strip.getPixelColor
        self.getPixelColorRGB = self.main_strip.getPixelColorRGB
        self.getPixelColorRGBW = self.main_strip.getPixelColorRGBW
        self.off = self.main_strip.off

        # Substitute for __del__, traps an exit condition and cleans up properly
        atexit.register(self._cleanup)

    def __getitem__(self, pos):
        """Return the 24-bit RGB color value at the provided position or slice
        of positions.
        """
        return self._values[pos]
        # alternative, if returning numpy arrays is not desired:
        #ret = self._values[pos]
        #if type(ret) is np.ndarray:
        #    return list(ret)
        #return ret

    def __setitem__(self, pos, value):
        """Set the 24-bit RGB color value at the provided position or slice of
        positions. If value is a slice it is zip()'ed with pos to set as many
        leds as there are values.
        """
        # Handle if a slice of positions are passed in by setting the appropriate
        # LED data values to the provided value.
        # Cast to int() as value may be a numpy non-int value.
        self._values[pos]=value

    def __len__(self):
        return ws.ws2811_channel_t_count_get(self._channel)

    def numPixels(self):
        """Return the number of pixels in the display."""
        return len(self)

    def _cleanup(self):
        # Clean up memory used by the library when not needed anymore.
        if self._leds is not None:
            ws.ws2811_fini(self._leds)
            ws.delete_ws2811_t(self._leds)
            self._leds = None
            self._channel = None

    def setGamma(self, gamma):
        if type(gamma) is list and len(gamma) == 256:
            ws.ws2811_channel_t_gamma_set(self._channel, gamma)

    def begin(self):
        """Initialize library, must be called once before other functions are
        called.
        """
        resp = ws.ws2811_init(self._leds)
        if resp != 0:
            str_resp = ws.ws2811_get_return_t_str(resp)
            raise RuntimeError('ws2811_init failed with code {0} ({1})'.format(resp, str_resp))
        # initialize array view
        self._values = ws.ws2811_array_get(self._channel)
        self._colors = self._values.view(dtype=np.uint8).reshape((-1,4)) # get view of individual color bytes
        if self._values.dtype.byteorder == "<" or (self._values.dtype.byteorder == "=" and sys.byteorder == "little"):
            self._colors = self._colors[:,::-1]

    def show(self):
        """Update the display with the data from the LED buffer."""
        resp = ws.ws2811_render(self._leds)
        if resp != 0:
            str_resp = ws.ws2811_get_return_t_str(resp)
            raise RuntimeError('ws2811_render failed with code {0} ({1})'.format(resp, str_resp))

    def getPixels(self):
        return self._values

    def getSubPixels(self):
        return self._colors

    def createPixelSubStrip(self, s=None, stop=None, step=None):
        """Create a PixelSubStrip starting with pixel `first`
        Either specify the `num` of pixels or the `last` pixel.

        All the methods of a PixelSubStrip are available on PixelStrip
        objects.

        Note: PixelSubStrips are not prevented from overlappping
        """
        if isinstance(s,slice):
            return self.PixelSubStrip(self, s)
        elif stop is not None:
            return self.PixelSubStrip(self, slice(s, stop, step))
        else:
            return self.PixelSubStrip(self, slice(s)) # s is interpreted as "stop", ie. last element in this case - this is a breaking change
            # this ensures the same behavior as expected from slice() or range() (slice(5)==slice(0,5) != slice(5,None))
            # to consistently use the first parameter as "start" instead, always use slice(s,stop,step)


    class PixelSubStrip:
        """A PixelSubStrip handles a subset of the pixels in a PixelStrip

        strip = PixelStrip(...)
        strip1 = strip.createPixelSubStrip(0, num=10)  # controls first 10 pixels
        strip2 = strip.createPixelSubStrip(10, num=10)  # controls next 10 pixels

        strip2[5] will access the 15th pixel
        """

        def __init__(self, strip, s):
            self.strip = strip
            if not isinstance(s, slice):
                raise InvalidStrip(f"invalid substrip specification - use createPixelSubStrip or pass a slice object")
            try:
                self._values = strip.getPixels()[s]
                self._colors = strip.getSubPixels()[s]
            except Exception as E:
                raise InvalidStrip(f"invalid index: {E=}")

        def __len__(self):
            return len(self._values)


        def __getitem__(self, pos):
            """Return the 24-bit RGB color value at the provided position or slice
            of positions.
            """
            return self._values[pos]

        def __setitem__(self, pos, value):
            """Set the 24-bit RGB color value at the provided position or slice of
            positions. If value is a slice it is zip()'ed with pos to set as many
            leds as there are values.
            """
            self._values[pos] = value

        def setPixelColor(self, n, color):
            """Set LED at position n to the provided 24-bit color value (in RGB order).
            If n is a slice then color can be a value which is repeated for all leds
            or a slice of values which are applied to the leds.
            """
            self[n] = color

        def setPixelColorRGB(self, n, red, green, blue, white=0):
            """Set LED at position n to the provided red, green, and blue color.
            Each color component should be a value from 0 to 255 (where 0 is the
            lowest intensity and 255 is the highest intensity).
            """
            # Translation to n done in setPixelColor
            self[n] = Color(red, green, blue, white)

        def getBrightness(self):
            return ws.ws2811_channel_t_brightness_get(self.strip._channel)

        def setBrightness(self, brightness):
            """Scale each LED in the buffer by the provided brightness.  A brightness
            of 0 is the darkest and 255 is the brightest.

            This method affects all pixels in all PixelSubStrips.
            """
            ws.ws2811_channel_t_brightness_set(self.strip._channel, brightness)

        def getPixels(self):
            """Return an object which allows access to the LED display data as if
            it were a sequence of 24-bit RGB values.
            """
            return self._values # breaking change - now returns a reference instead of a copy

        def numPixels(self):
            """Return the number of pixels in the strip."""
            return len(self._values)

        def getPixelColor(self, n):
            """Get the 24-bit RGB color value for the LED at position n."""
            return self[n]

        def getPixelColorRGB(self, n):
            return _int_to_RGBW(self[n])

        def getPixelColorRGBW(self, n):
            return _int_to_RGBW(self[n])

        def show(self):
            self.strip.show()

        def off(self):
            self[:] = 0
            self.strip.show()

class InvalidStrip(Exception):
    pass

# Shim for back-compatibility
class Adafruit_NeoPixel(PixelStrip):
    pass
