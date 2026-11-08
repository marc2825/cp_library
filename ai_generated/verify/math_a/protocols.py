"""Independent Python integer, permutation and interval-DP oracles."""
import math
import random

MOD=998244353
LO=-(1<<63)
HI=(1<<63)-1

def verify(run,binary,name,rng):
    if name=='floor_sum':
        cases=[(rng.randrange(25),rng.randrange(1,25),rng.randrange(-100,101),rng.randrange(-100,101)) for _ in range(12)]
        out=run([binary],input=str(len(cases))+'\n'+'\n'.join(' '.join(map(str,c)) for c in cases)+'\n')
        assert list(map(int,out.split()))==[sum((a*i+b)//m for i in range(n)) for n,m,a,b in cases]
    elif name=='euler_phi':
        n=rng.randrange(1,10000)
        assert int(run([binary],input=f'{n}\n'))==sum(math.gcd(n,k)==1 for k in range(1,n+1))
    elif name=='primitive_root':
        primes=[p for p in range(2,300) if all(p%d for d in range(2,math.isqrt(p)+1))]
        data=rng.sample(primes,10);out=list(map(int,run([binary],input='10\n'+'\n'.join(map(str,data))+'\n').split()))
        for p,g in zip(data,out):
            assert {pow(g,k,p) for k in range(p-1)}==set(range(1,p))
    elif name=='derangements':
        n=rng.randrange(1,70);m=rng.choice([1,8,97,1000000007,HI]);fact=1;answer=[]
        # Inclusion-exclusion n! sum (-1)^k/k!, computed with exact integers.
        for i in range(1,n+1):
            fact*=i;answer.append(sum((-1)**k*(fact//math.factorial(k)) for k in range(i+1))%m)
        assert list(map(int,run([binary],input=f'{n} {m}\n').split()))==answer
    elif name=='partition_numbers':
        n=rng.randrange(80);a=[1]+[0]*n
        for part in range(1,n+1):
            for total in range(part,n+1):a[total]=(a[total]+a[total-part])%MOD
        assert list(map(int,run([binary],input=f'{n}\n').split()))==a
    elif name=='knuth':
        n=rng.randrange(1,20);a=[rng.randrange(30) for _ in range(n)];dp=[[0]*(n+1) for _ in range(n+1)]
        for length in range(2,n+1):
            for l in range(n-length+1):
                r=l+length;dp[l][r]=sum(a[l:r])+min(dp[l][k]+dp[k][r] for k in range(l+1,r))
        assert int(run([binary],input=str(n)+'\n'+' '.join(map(str,a))+'\n'))==dp[0][n]
    else:
        n=rng.randrange(1,40);c=rng.randrange(100);h=[rng.randrange(1,500) for _ in range(n)];dp=[0]*n
        for i in range(1,n):dp[i]=min(dp[j]+(h[i]-h[j])**2+c for j in range(i))
        assert int(run([binary],input=f'{n} {c}\n'+' '.join(map(str,h))+'\n'))==dp[-1]

def python_floor(n,m,a,b):
    qa,a=divmod(a,m);qb,b=divmod(b,m);first=n*(n-1)//2*qa;second=n*qb
    if not all(LO<=x<=HI for x in [first,second,first+second]):return 'overflow'
    base=first+second;answer=0
    while True:
        q,a=divmod(a,m);answer+=n*(n-1)//2*q
        q,b=divmod(b,m);answer+=n*q
        y=a*n+b
        if y<m:break
        n,b=divmod(y,m);m,a=a,m
    result=base+answer
    return str(result) if LO<=result<=HI else 'overflow'

def python_crt(residues,moduli):
    r,m=0,1
    for b,n in zip(residues,moduli):
        b%=n;g=math.gcd(m,n)
        if (b-r)%g:return 'none'
        if m*(n//g)>HI:return 'overflow'
        t=((b-r)//g*pow(m//g,-1,n//g))%(n//g);r+=m*t;m*=n//g
    return f'{r} {m}'

def verify_big_integers(run,binary):
    rng=random.Random(314159);lines=[];expected=[]
    for _ in range(2200):
        n=rng.choice([0,1,2,4294967295,rng.randrange(1<<32)])
        m=rng.choice([1,2,4294967295,rng.randrange(1,1<<32)])
        a=rng.choice([LO,HI,-1,0,rng.randrange(LO,HI)]);b=rng.choice([LO,HI,-1,0,rng.randrange(LO,HI)])
        lines.append(f'floor {n} {m} {a} {b}');expected.append('0' if not n else python_floor(n,m,a,b))
    for _ in range(1800):
        k=rng.randrange(8);m=[rng.choice([1,rng.randrange(1,100),rng.randrange(1,HI)]) for _ in range(k)];r=[rng.randrange(LO,HI) for _ in range(k)]
        lines.append('crt '+str(k)+' '+' '.join(f'{a} {b}' for a,b in zip(r,m)));expected.append(python_crt(r,m))
    choices=[1,2,3,5,7,11,97,1000000007,1000000009,2147483647,4294967291,2305843009213693951]
    for _ in range(1000):
        m=rng.sample(choices,rng.randrange(len(choices)+1));r=[rng.randrange(LO,HI) for _ in m];target=rng.choice(choices+[HI]);x=0;product=1
        for a,b in zip(r,m):
            x+=product*((a-x)*pow(product,-1,b)%b);product*=b
        lines.append('garner '+str(len(m))+' '+' '.join(f'{a} {b}' for a,b in zip(r,m))+' '+str(target));expected.append(str(x%target))
    actual=run([binary],input='\n'.join(lines)+'\n').splitlines()
    for i,(a,b) in enumerate(zip(actual,expected)):
        assert a==b,(lines[i],a,b)
    assert len(actual)==len(expected)
    print(f'{len(lines)} arbitrary-precision floor/CRT/Garner cases passed',flush=True)
