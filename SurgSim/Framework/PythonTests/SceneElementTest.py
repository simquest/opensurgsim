import unittest
from Framework import *

class SceneElementTestCase(unittest.TestCase) :

	def test_instantation(self) :
		se = SceneElement("Test")
		self.assertEqual("Test",se.getName())
		

if __name__ == '__main__':
    unittest.main()