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

class _PixelStripBase:
    def __init__(self):
        raise TypeError("this class is abstract") # is this the correct error type for this?

    # mutual methods for PixelStrip and PixelSubStrip
    def __getitem__(self, pos):
        """Return the 24-bit RGB color value at the provided position or slice
        of positions.
        """
        return self._view[pos]

    def __setitem__(self, pos, value):
        """Set the 24-bit RGB color value at the provided position or slice of
        positions. If value is a slice it is zip()'ed with pos to set as many
        leds as there are values.
        """
        self._view[pos]=value

    def __len__(self):
        return self._view.shape[0]

    def setPixelColor(self, n, color):
        """Set LED at position n to the provided 24-bit color value (in RGB order).
        If n is a slice then color can be a value which is repeated for all leds
        or a slice of values which are applied to the leds.
        """
        self._view[n] = color

    def setPixelColorRGB(self, n, red, green, blue, white=0):
        """Set LED at position n to the provided red, green, and blue color.
        Each color component should be a value from 0 to 255 (where 0 is the
        lowest intensity and 255 is the highest intensity).
        """
        self.setPixelColor(n, Color(red, green, blue, white))

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
        return self._view

    def getSubPixels(self):
        return self._colors

    def numPixels(self):
        """Return the number of pixels in the strip."""
        return len(self)
        # alternative: np.product(self._view.shape)
        # would be identical unless called on a matrix substrip; in that case,
        # the latter would return the total number of pixels instead of the first dimension

    def getPixelColor(self, n):
        """Get the 24-bit RGB color value for the LED(s) at index n."""
        return self._view[n]

    def getPixelColorRGB(self, n):
        # return single RGBW for int index or array of RGBW for slice/advanced indexing
        return _int_to_RGBW(self._view[n])
        # alternative: it might be convenient for users if instead of returning arrays of RGBW for slices,
        # it returned an RGBW object representing the "color array" - allowing things
        # like arr.r[1:4] or arr[1:4].r instead of np.vectorize(lambda x:x.r)(arr[1:4])

    def getPixelColorRGBW(self, n):
        return _int_to_RGBW(self._view[n])

    def show(self):
        self.strip.show()

    def off(self):
        self._view[:] = 0
        self.strip.show()

class PixelStrip(_PixelStripBase):
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

        # set numpy views to empty arrays; "index out of bounds" seems marginally more
        # helpful than "variable doesn't exist" when accessing methods before begin()
        self._view = np.zeros((0))
        self._colors = np.zeros((0,4))
        # alternative: set it to a "python-backed" numpy array of the correct size and copy over the content in begin()
        # this would have the advantage of __len__ being fully backwards compatible even in the edge case of accessing len(strip) before strip.begin()

        # Substitute for __del__, traps an exit condition and cleans up properly
        atexit.register(self._cleanup)

    #def __len__(self):
    #    return ws.ws2811_channel_t_count_get(self._channel)

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
        self._view = ws.ws2811_array_get(self._channel)
        # get view of individual color bytes, respecting architecture dependant byteorder
        self._colors = self._view.view(dtype=np.uint8).reshape((-1,4))
        if self._view.dtype.byteorder == "<" or (self._view.dtype.byteorder == "=" and sys.byteorder == "little"):
            self._colors = self._colors[:,::-1]

    def show(self):
        """Update the display with the data from the LED buffer."""
        resp = ws.ws2811_render(self._leds)
        if resp != 0:
            str_resp = ws.ws2811_get_return_t_str(resp)
            raise RuntimeError('ws2811_render failed with code {0} ({1})'.format(resp, str_resp))

    def createPixelSubStrip(self, first, last=None, num=None):
        """Create a PixelSubStrip starting with pixel `first`
        Either specify the `num` of pixels or the `last` pixel.

        All the methods of a PixelSubStrip are available on PixelStrip
        objects.

        Note: PixelSubStrips are not prevented from overlappping
        """
        # TODO: if doing any substrip boundary validation, should probably also check that last>=first (/num >=0) and first >=0
        if last:
            if last > len(self):
                raise self.InvalidStrip(f"Too many pixels ({last})."
                                        f"Strip only has {len(self)}.")
            return self.PixelSubStrip(self, first, last=last)
        if num:
            if first + num > len(self):
                raise self.InvalidStrip(f"Too many pixels ({first + num})."
                                        f"Strip only has {len(self)}.")
            return self.PixelSubStrip(self, first, num=num)
        #raise self.InvalidStrip("Need num or last to create a PixelSubStrip")
        # allow only settings first as shorthand for "to end of strip"
        return self.PixelSubStrip(self, first)

    class InvalidStrip(Exception):
        pass

    class PixelSubStrip(_PixelStripBase):
        """A PixelSubStrip handles a subset of the pixels in a PixelStrip

        strip = PixelStrip(...)
        strip1 = strip.createPixelSubStrip(0, num=10)  # controls first 10 pixels
        strip2 = strip.createPixelSubStrip(10, num=10)  # controls next 10 pixels

        strip2[5] will access the 15th pixel
        """
        def __init__(self, strip, first, last=None, num=None):
            # TODO: strides, reshape
            self.strip = strip
            if first is None:
                first = 0
            if last is None:
                if num is not None:
                    last = first + num
                else:
                    last = len(strip)
            self._view = strip._values[first:last]
            self._colors = strip._colors[first:last]
            # assuming no lasts means "until end of strip", similar to indexing (a[5:])
            # raise self.InvalidStrip("Must specify number or last pixel to "
            #                            "create a PixelSubStrip")

        # all other methods are inherited from _PixelStripBase

# Shim for back-compatibility
class Adafruit_NeoPixel(PixelStrip):
    pass
