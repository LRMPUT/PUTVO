#ifndef LCG_H
#define LCG_H

class LCG
{
      int seed;
      int n;
      int prime;
      int curr;
public:
    LCG(int seed, int n);
    int GetPrime(int n);
    bool IsPrime(int n);
    int NextInt();
    int ModuloOfSum(int a, int b, int m);
};

#endif // LCG_H
