#include "lcg.h"
#include <assert.h>
#include <math.h>

LCG::LCG(int seedx, int nx)
{
    seed = seedx;
    n = nx;
    if(n>1){
        prime = GetPrime(n);
        assert(prime!=-1);
        curr = seed % n;
    }
    else{
        prime=1;
        curr=0;
    }
}

int LCG::GetPrime(int nx)
{
    int p = (nx / 3);
    while (p < nx)
    {
        if (IsPrime(p) && nx% p != 0)
            return p;
        ++p;
    }
    return -1; // error
}

bool LCG::IsPrime(int nx) // helper for GetPrime
{
    int divisor = 2;
    int maxDivisor = (int)(sqrt(nx + 1));

    while (divisor < maxDivisor)
    {
        if (nx % divisor == 0)
            return false;
        ++divisor;
    }
    return true;
}

int LCG::NextInt()
{
    if(n<=1)
    {
        return 0;
    }
    curr = ModuloOfSum(curr, prime, n);
    return curr;
}

int LCG::ModuloOfSum(int a, int b, int m)
{
    int mod1 = a % m;
    int mod2 = b % m;
    return (mod1 + mod2) % m;
}
