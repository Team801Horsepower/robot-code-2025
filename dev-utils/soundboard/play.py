from sys import argv
import simpleaudio as sa

wave_obj = sa.WaveObject.from_wave_file(argv[1])
play_obj = wave_obj.play()
play_obj.wait_done()
