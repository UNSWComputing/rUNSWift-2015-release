from Task import BehaviourTask


class GameController(BehaviourTask):

   def init(self):
      msg = '\n'.join((
         '',
         '# Congratulations runswift is running.',
         '# Remember to make the robot stiff (or limp) with ',
         '# two chest button presses, the robot should say "body stiff". ',
         '# ',
         '# You might like to try a specific skill, for example: ',
         '#     runswift -s WalkInACircle',
      ))
      print(msg)

   def transition(self):
      pass

   def _tick(self):
      pass
