import unittest,random
from mechanism_lab.blender_volume import coefficients
class Tests(unittest.TestCase):
 def test_exact_extinction_and_scattering(self):
  r=random.Random(17)
  for _ in range(1000):
   a=[r.random() for _ in range(3)];s=[r.random() for _ in range(3)];ad=r.uniform(-5,100);sd=r.uniform(-5,100)
   sa,ss=coefficients(a,ad,s,sd)
   for j in range(3):
    self.assertEqual(ss[j],s[j]*max(sd,0))
    self.assertEqual(sa[j]+ss[j],(1-a[j])*max(ad,0)+s[j]*max(sd,0))
 def test_invalid_rejected(self):
  for bad in (float('nan'),float('inf'),-0.1,1.1):
   with self.assertRaises(ValueError):coefficients([bad,0,0],1,[1,1,1],1)
if __name__=='__main__':unittest.main()
