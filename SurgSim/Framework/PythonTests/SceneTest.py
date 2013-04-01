import unittest
from Framework import *

class SceneTestCase(unittest.TestCase) :

	def test_instantation(self) :
		scene = Scene()

	def test_addSceneElement(self) :
		scene = Scene()
		se = SceneElement("Test")
		self.assertEqual(True, scene.addSceneElement(se))
		self.assertEqual(False, scene.addSceneElement(se))

if __name__ == '__main__':
    unittest.main()