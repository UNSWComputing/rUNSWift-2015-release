# -*- coding: utf-8 -*-
"""
Based on:
1.  Alexandre Mazel Stack Overflow answer.
    http://stackoverflow.com/questions/24243757/nao-robot-remote-audio-problems
2.  Austrian Kangaroos whistle detection implementation,
    by Thomas Hamboeck <th@complang.tuwien.ac.at>, 2014.
    http://www.informatik.uni-bremen.de/spl/bin/view/Website/OpenSource
    http://www.austrian-kangaroos.com/public/code/WhistleDetector.tgz

This records audio on the robot. The Stack Overflow original claims to stream
from the robot to the PC so you wouldn't need the following process to verify
it works:

0.  PC$ Update code, make NAO_RECORD_ALL_AUDIO = True
1.  PC$ nao_sync hunter
2.  robot$ cd data/behaviours/audio      # TODO: Should move out of behaviours to make future code releases easier
3.  robot$ python whistle_detector.py
4.  Say something.
5.  robot$ <Ctrl-C>
6.  PC$ scp nao@hunter:data/behaviours/audio/saved_sound.wav saved_sound.wav
7.  Open the file in your PCM WAV audio player of choice :)
"""
import os

###################################################################
# Developer configurable settings - you may wish to play with these
###################################################################

# Set verbosity to between 0-3 to print less or more information
VERBOSITY = 1
NAO_RECORD_ALL_AUDIO = False
NAO_SAVE_WHISTLE = True
NAO_WHISTLE_LOCATION = os.path.join(os.environ['HOME'], 'whistle')
NAO_WHISTLES_TO_KEEP = 20
WHISTLE_FILE_FORMAT = 'whistle_%Y_%m_%d_%H%M%S.wav'


class Config(object):
   class ConfigError(ValueError):
      pass

   def __init__(self):
      # Values chosen as they correspond to the Audacity
      # spectrogram of recorded whistles
      self.fWhistleBegin = 2000
      self.fWhistleEnd = 4000
      self.fSampleRate = 48000

      self.background_threshold = 0.7
      self.spectrum_threshold = 2.5
      self.temporal_medians_threshold = 5.0
      self.nWhistleOkaySpectra = 12  # ~250ms at 48 kHz
      self.nWhistleMissSpectra = 4  # ~83ms at 48 kHz

      # window_size of 1024 chosen to get 46.875 sps = spectra per second
      self.spectra_per_second = 47
      self.window_size = 1024
      self.sample_rate = self.spectra_per_second * self.window_size

      # Derived config values
      self.spectrum_whistle_begin = (
         self.fWhistleBegin * self.window_size
         // self.fSampleRate
      )
      self.spectrum_whistle_end = (
         self.fWhistleEnd * self.window_size
         // self.fSampleRate
      )

      # Config validation
      upper_bound = self.fSampleRate / 2
      if self.fWhistleBegin < 0:
         raise self.ConfigError('fWhistleBegin is below 0')
      if self.fWhistleBegin > upper_bound:
         raise self.ConfigError('fWhistleBegin is above Nyquist frequency')
      if self.fWhistleEnd < 0:
         raise self.ConfigError('fWhistleEnd is below 0')
      if self.fWhistleEnd > upper_bound:
         raise self.ConfigError('fWhistleEnd is above Nyquist frequency')
      if self.fWhistleBegin >= self.fWhistleEnd:
         raise self.ConfigError('fWhistleBegin must be below fWhistleEnd')


_config = Config()

###################################################################
# End Developer configurable settings
###################################################################


try:
   # Workaround for not having pip to install Python libraries & packages
   from wtb_pip import alsaaudio
except ImportError:
   try:
      import alsaaudio
   except ImportError:
      alsaaudio = None

try:
   from naoqi import ALProxy

   colored = lambda a, b: a  # Fix VERBOSITY=3 on nao
   ON_NAO_ROBOT = True
except ImportError:
   # For OSX/Ubuntu testing, there is no naoqi, but there are terminal colours
   ALProxy = None
   from termcolor import colored

   print('Warning: Could not import naoqi components')
   ON_NAO_ROBOT = False

import numpy
import subprocess
import time
import traceback
import wave
from datetime import datetime
from sys import exit, stdout


def check_runswift_running():
   cmd1 = subprocess.Popen(
      ['ps', 'ax'],
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE,
   )
   data = cmd1.communicate()[0]
   lines = data.split('\n')
   has_runswift = [l for l in lines if 'runswift' in l]
   if len(has_runswift) == 0:
      if VERBOSITY >= 2:
         print('whistle_detector: exiting as runswift not running')
      exit(0)
   return True


def timethis(func):
   """
   Very basic version. Feel free to improve :)
   """
   def wrap(*args, **kwargs):
      start = time.time()
      try:
         result = func(*args, **kwargs)
         return result
      finally:
         if VERBOSITY >= 2:
            msg = '{}: Took {:.3f} seconds'.format(
               func.__name__,
               time.time() - start,
            )
            print(msg)
   return wrap


def whistle_action():
   # Callback chains can get messy so this is now just for debugging
   if VERBOSITY >= 2:
      print('!!! Whistle heard !!!')


def debug_vis(spectrum, mean, st_dev):
   if not VERBOSITY >= 3:
      return
   # Debugging spectrum
   # NOTE DOES NOT NECESSARILY AGREE WITH CONFIGURED SETTINGS
   buckets = len(spectrum) // 40
   as_buckets = [sum(spectrum[i * buckets:(i + 1) * buckets]) // buckets
                 for i in range(len(spectrum) // buckets)]
   stdout.write('\n__')
   for i, got in enumerate(as_buckets):
      lower = _config.spectrum_whistle_begin // buckets
      upper = _config.spectrum_whistle_end // buckets
      color = 'white'
      if lower < i < upper:
         color = 'green'
      if got < mean + (_config.temporal_medians_threshold - 1) * st_dev:
         stdout.write(' ')
      elif got < mean + _config.temporal_medians_threshold * st_dev:
         stdout.write(colored(':', color))
      else:
         stdout.write(colored('|', color))
   stdout.write('__\n')


class WhistleState(object):
   def __init__(self):
      self.whistleCounter = 0
      self.whistleMissCounter = 0
      self.whistleDone = False
      # Remember the last second of data
      self.stats_memory = []
      self.stats_remember = _config.spectra_per_second

   def interrogate(self, complex_spectrum):
      # Austrian Kangaroos algorithm assumes positive spectrum values
      spectrum = [abs(a) for a in complex_spectrum]

      spectrum_mean = numpy.mean(spectrum)
      spectrum_st_dev = numpy.std(spectrum)
      self.stats_memory.append((spectrum_mean, spectrum_st_dev))
      if len(self.stats_memory) > self.stats_remember:
         self.stats_memory.pop(0)
      else:
         # Don't continue until we have at least self.stats_remember,
         # i.e. a significant amount of data to work with
         return

      means, devs = [], []
      for mu, sigma in self.stats_memory:
         means.append(mu)
         devs.append(sigma)
      last_second_mean = numpy.median(means)
      last_second_st_dev = numpy.median(devs)

      spectrum_threshold = (
         spectrum_mean +
         _config.spectrum_threshold * spectrum_st_dev
      )
      temporal_threshold = (
         last_second_mean +
         _config.temporal_medians_threshold * last_second_st_dev
      )
      whistle_threshold = max(spectrum_threshold, temporal_threshold)
      if VERBOSITY >= 2:
         stdout.write('.')
         stdout.flush()

      debug_vis(spectrum, last_second_mean, last_second_st_dev)

      begin = _config.spectrum_whistle_begin
      end = _config.spectrum_whistle_end

      # Adaptively "grow" background noise zones, so we
      # increase begin and/or decrease end, potentially
      # discarding the entire spectrum where there isn't a significant noise.
      background_growth_threshold = (
         spectrum_mean +
         _config.background_threshold * spectrum_st_dev
      )
      # About (4000 - 2000) / 10 = 200 MHz per "bucket" at this time
      num_buckets = 10
      grow_size = (end - begin) / num_buckets
      for _ in range(num_buckets):
         bucket_mean = numpy.mean(spectrum[begin:begin+grow_size])
         if bucket_mean < background_growth_threshold:
            begin += grow_size
         else:
            break
      for _ in range(num_buckets):
         bucket_mean = numpy.mean(spectrum[end-grow_size:end])
         if bucket_mean < background_growth_threshold:
            end -= grow_size
         else:
            break

      # Actually filter the spectrum
      filtered = spectrum[begin:end]
      found = False
      if filtered:
         filtered_mean = numpy.mean(filtered)
         found = bool(filtered_mean > whistle_threshold)

      # Thanks Austrian Kangaroos for the starting algorithm.
      state = self
      if state.whistleDone:
         if VERBOSITY >= 2:
            print("whistleDone")
         if not found:
            state.whistleMissCounter += 1
            if state.whistleMissCounter > _config.nWhistleMissSpectra:
               state.reset()
      else:
         if found:
            if VERBOSITY >= 2:
               print("found, not done")
               print("C {} - MC {}".format(state.whistleCounter,
                                           state.whistleMissCounter))
            state.whistleCounter += 1
            state.whistleMissCounter = 0
         elif state.whistleCounter > 0:
            if VERBOSITY >= 2:
               print("inc MissCounter")
               print("C {} - MC {}".format(state.whistleCounter,
                                           state.whistleMissCounter))
            state.whistleMissCounter += 1
            if state.whistleMissCounter > _config.nWhistleMissSpectra:
               state.reset()
         if state.whistleCounter >= _config.nWhistleOkaySpectra:
            whistle_action()
            state.reset()
            state.whistleDone = True

   def reset(self):
      self.whistleCounter = 0
      self.whistleMissCounter = 0
      self.whistleDone = False


class SoundReceiverMixin(object):
   """
   Mixin to abstract out functionality for testing on both PC and Nao.
   """

   def __init__(self):
      self.out_file = None
      self.state = WhistleState()
      self.sound_time = 0.0
      self.whistle_last_saved = (0, 0)
      self.last_2_secs_buffers = []

   def find_whistle(self, sound_data, num_channels, time_stamp=None):
      signal = sound_data[0]

      # Split signal into chunks of window_size,
      # NOTE: Discarding last 192 bytes of 8192 naoqi sends...
      signal = signal[:len(signal) - len(signal) % _config.window_size]
      processed = 0
      while processed < len(signal):
         window = signal[processed:processed + _config.window_size]
         processed += _config.window_size
         spectrum = numpy.fft.rfft(window)
         self.state.interrogate(spectrum)
         if time_stamp and self.state.whistleDone:
            self.save_whistle_to_file(num_channels, time_stamp)

         if VERBOSITY >= 3:
            print('{:.3f} seconds'.format(self.sound_time /
                                          _config.spectra_per_second))
         self.sound_time += 1.0

   def save_whistle_to_file(self, num_channels, time_stamp):
      raise NotImplementedError('See subclasses')

   @staticmethod
   def to_sound_data(audio_buffer, num_channels, samples_per_channel):
      sound_data_interlaced = numpy.fromstring(
         str(audio_buffer),
         dtype=numpy.int16)
      sound_data = numpy.reshape(
         sound_data_interlaced,
         (num_channels, samples_per_channel),
         'F')
      return sound_data


if ON_NAO_ROBOT:
   class SoundReceiverModule(SoundReceiverMixin):
      def __init__(self):
         super(SoundReceiverModule, self).__init__()
         pcm = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE)
         pcm.setrate(_config.sample_rate)
         pcm.setchannels(1)
         pcm.setperiodsize(_config.window_size)
         if VERBOSITY >= 2:
            print(pcm.dumpinfo())
         self.pcm = pcm

      def listen_forever(self):
         count = 0
         while True:
            count += 1
            samples_per_channel, audio_buffer = self.pcm.read()
            buffer_len = len(audio_buffer)
            if buffer_len < 100:
               # Skip over bad audio buffers we sometimes get
               continue
            if buffer_len % 2 != 0:
               # Discard last byte if we get an odd number of bytes as
               # numpy won't reshape it properly
               audio_buffer = audio_buffer[:-1]
            self.process_buffer(
               num_channels=1,
               samples_per_channel=samples_per_channel,
               time_stamp=get_aldebaran_timestamp(),
               audio_buffer=audio_buffer
            )
            # This is slow so don't do it too often
            if count % 30 == 0:
               check_runswift_running()

      def process_buffer(self, num_channels, samples_per_channel,
                         time_stamp, audio_buffer):
         """
         This is THE method that receives all the sound buffers
         from the "ALAudioDevice" module
         """
         keep_seconds = 2
         encoded_amplitude = 2  # Assume 16-bit instead of 24- or 32-bit
         bytes_so_far = sum([len(b) for b in self.last_2_secs_buffers])
         keep = (_config.sample_rate * keep_seconds *
                 num_channels * encoded_amplitude)
         if bytes_so_far > keep:
            self.last_2_secs_buffers.pop(0)
         self.last_2_secs_buffers.append(audio_buffer)

         sound_data = self.to_sound_data(audio_buffer,
                                         num_channels,
                                         samples_per_channel)
         self.find_whistle(sound_data, num_channels, time_stamp)
         if NAO_RECORD_ALL_AUDIO:
            # save to file
            if self.out_file is None:
               out_file_name = 'saved_sound.wav'
               print("INF: Writing sound to '%s'" % out_file_name)
               try:
                  os.remove(out_file_name)
               except OSError:
                  pass
               out_file = wave.open(out_file_name, 'wb')
               out_file.setnchannels(num_channels)
               out_file.setframerate(_config.sample_rate)
               # Assume we record 16-bit, or 2-Byte audio
               out_file.setsampwidth(2)
               self.out_file = out_file
            try:
               self.out_file.writeframesraw(audio_buffer)
            except AttributeError:
               # Ignore if file already closed
               pass

      def save_whistle_to_file(self, num_channels, time_stamp):
         seconds, us = time_stamp
         if seconds - 5 < self.whistle_last_saved[0]:
            # Don't save more than once every 5 seconds
            return
         self.whistle_last_saved = time_stamp

         # Ensure folder exists
         if not os.path.exists(NAO_WHISTLE_LOCATION):
            os.makedirs(NAO_WHISTLE_LOCATION)

         if NAO_WHISTLES_TO_KEEP:
            # Remove files so we don't go over the limit
            files = sorted(os.listdir(NAO_WHISTLE_LOCATION))
            for old_file in files[:-(NAO_WHISTLES_TO_KEEP - 1)]:
               old = os.path.join(NAO_WHISTLE_LOCATION, old_file)
               if VERBOSITY >= 2:
                  print('Removing file: {}'.format(old))
               os.remove(old)

         if NAO_SAVE_WHISTLE:
            dt = datetime.utcfromtimestamp(time_stamp[0])
            new_file_name = dt.strftime(WHISTLE_FILE_FORMAT)
            new_path = os.path.join(NAO_WHISTLE_LOCATION, new_file_name)

            if VERBOSITY >= 2:
               print('Saving whistle as: {}'.format(new_path))
            out_file = wave.open(new_path, 'wb')
            out_file.setnchannels(num_channels)
            out_file.setframerate(_config.sample_rate)
            # Assume we record 16-bit, or 2-Byte audio
            out_file.setsampwidth(2)
            for audio_buffer in self.last_2_secs_buffers:
               out_file.writeframes(audio_buffer)
            out_file.close()


def get_aldebaran_timestamp():
   # Quick and dirty way to rebuild timestamp format Aldebaran sends
   t = time.time()
   seconds = int(t)
   u_seconds = '{:0.6f}'.format(t % 1)[2:]
   timestamp = (seconds, u_seconds)
   return timestamp


@timethis
def run_on_nao():
   detector = SoundReceiverModule()
   detector.listen_forever()


class PCSoundReceiverModule(SoundReceiverMixin):
   """
   Place to collect any PC-specific stuff.
   """

   def save_whistle_to_file(self, num_channels, time_stamp):
      # No point saving the whistle on a PC where we run regression tests.
      return

   def wav_test(self, in_file_name):
      """
      Testing if robot audio saved in a WAV file, when read back in on PC,
      is representative of a whistle.
      """
      num_channels = 1
      samples_per_channel = _config.window_size
      in_file = wave.open(in_file_name, 'rb')
      read = 0
      total_frames = in_file.getnframes()
      while read < total_frames:
         read += _config.window_size
         audio_buffer = in_file.readframes(_config.window_size)
         if len(audio_buffer) < _config.window_size:
            if VERBOSITY >= 2:
               print('Warning: Skipping unexpected buffer size {}'
                     .format(len(audio_buffer)))
            continue
         sound_data = self.to_sound_data(audio_buffer,
                                         num_channels,
                                         samples_per_channel)
         self.find_whistle(sound_data, num_channels)

         if self.state.whistleDone:
            # Whistle found
            return True
      # No whistle found
      return False


@timethis
def pc_wav_test():
   success = 0
   false_negative = 0
   false_positive = 0

   default = os.path.join(os.environ['HOME'], 'Projects/rUNSWift')
   runswift_dir = os.environ.get('RUNSWIFT_CHECKOUT_DIR', default)
   yes_dir = os.path.join(runswift_dir, 'test/audio/whistle_yes')
   for filename in sorted(os.listdir(yes_dir)):
      if not filename.endswith('.wav'):
         continue
      file_path = os.path.join(yes_dir, filename)
      instance = PCSoundReceiverModule()
      test_result = instance.wav_test(in_file_name=file_path)
      if test_result is True:
         print(colored('OK: ' + filename, 'green'))
         success += 1
      else:
         print(colored('FALSE NEGATIVE: ' + filename, 'red'))
         false_negative += 1

   print('-' * 80)
   no_dir = os.path.join(runswift_dir, 'test/audio/whistle_no')
   for filename in sorted(os.listdir(no_dir)):
      if not filename.endswith('.wav'):
         continue
      file_path = os.path.join(no_dir, filename)
      instance = PCSoundReceiverModule()
      test_result = instance.wav_test(in_file_name=file_path)
      if test_result is False:
         print(colored('OK: ' + filename, 'green'))
         success += 1
      else:
         print(colored('FALSE POSITIVE: ' + filename, 'red'))
         false_positive += 1
   msg = ['run={}'.format(success + false_negative + false_positive)]
   color = 'yellow'
   if success:
      msg.append('success={}'.format(success))
      color = 'green'
   if false_negative:
      msg.append('false negatives={}'.format(false_negative))
      color = 'red'
   if false_positive:
      msg.append('false positives={}'.format(false_positive))
      color = 'red'
   print(colored('-' * 80, color))
   print(colored('TEST RESULTS: {}'.format(', '.join(msg)), color))
   print(colored('-' * 80, color))


def whistle_heard(num_seconds):
   """
   Reference implementation for C++ GameController::whistleHeard function.
   :return: True if a whistle file was created in the last num_seconds.
   """
   now = datetime.now()

   # Ensure folder exists
   if not os.path.exists(NAO_WHISTLE_LOCATION):
      os.makedirs(NAO_WHISTLE_LOCATION)
   file_names = sorted(os.listdir(NAO_WHISTLE_LOCATION))

   deltas = [
      now - datetime.strptime(file_name, WHISTLE_FILE_FORMAT)
      for file_name in file_names
   ]
   # Note: Check both abs() and non-abs() so we ignore future whistles
   return any(
      abs(delta.total_seconds()) < num_seconds and
      delta.total_seconds() < num_seconds
      for delta in deltas
   )


def kill_all_python_processes():
   # killall python processes https://stackoverflow.com/questions/18428
   # Updated to:
   # - Only kill whistle_detector processes
   # - Use a bash for loop to get around kill with no args printing stuff
   #os.system("kill $(ps aux | grep '[w]histle_detector' | awk '{print $2}')")
   os.system("kill $(sudo ps aux | grep [w]histle_detector | awk '{print $2}')")

def start_listening_for_whistles():
   # Start new background Python process to listen for whistles
   # and save detected whistle files in NAO_WHISTLE_LOCATION
   os.system('/usr/bin/amixer -qs < /home/nao/data/volumeinfo.dat')
   os.system('python $HOME/data/behaviours/audio/whistle_detector.py &')


if __name__ == '__main__':
   def write_to_file_with_stack_trace(message):
      if not os.path.exists(NAO_WHISTLE_LOCATION):
         os.makedirs(NAO_WHISTLE_LOCATION)
      with open(NAO_WHISTLE_LOCATION + '/whistle_log.txt', 'a') as outfile:
         outfile.write('-'*20 + datetime.now().isoformat() + '-'*20 + '\n')
         exc_name = getattr(getattr(e, '__class__', e), '__name__', '')
         outfile.write(exc_name + ': ' + unicode(e) + '\n')
         outfile.write(traceback.format_exc() + '\n')
         outfile.write(message + '\n')

   RETRY = 3
   while ON_NAO_ROBOT and RETRY > 0:
      RETRY -= 1
      try:
         run_on_nao()
      except alsaaudio.ALSAAudioError as e:
         # This worked on husker at the command line at least once
         # ... but added Luke's sudo thing to be safe.
         os.system('sudo killall pulseaudio')
         write_to_file_with_stack_trace(
            'Crash caught, attempting to kill '
            'pulseaudio to see if it fixes it.'
         )
      except Exception as e:
         write_to_file_with_stack_trace('Unknown crash detected ^_^\n')

   if not ON_NAO_ROBOT:
      pc_wav_test()
