import unittest

class Tests(unittest.TestCase):

    def test_angle(self):
        tests = [
            # z0, z1, expected_angle, is_cw
            (354, 10, 16, True),
            (354, 10, -344, False),
            (10, 354, 344, True),
            (10, 354, -16, False),
            (0, 179, 179, True),
            (0, 179, -181, False),
            (0, 181, 181, True),
            (0, 181, -179, False),
            (0, 400, 40, True),
        ]
        i = 0
        for z0, z1, expected_angle, is_cw in tests:
            i += 1
            print i
            if is_cw:
                actual_angle = (z1 - z0) % 360
            else:
                actual_angle = -((z0 - z1) % 360)
            self.assertEqual(actual_angle, expected_angle)

if __name__ == '__main__':
    unittest.main()
