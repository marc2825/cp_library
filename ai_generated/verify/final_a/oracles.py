"""Byte substrings, all Lyndon partitions, nim monomials, exact windows and search replay."""
from functools import lru_cache
from itertools import product
import random

MASK=(1<<64)-1
def splitmix(seed):
    while True:
        seed=(seed+0x9e3779b97f4a7c15)&MASK;z=seed;z=((z^(z>>30))*0xbf58476d1ce4e5b9)&MASK;z=((z^(z>>27))*0x94d049bb133111eb)&MASK
        yield z^(z>>31)

@lru_cache(None)
def nim_basis(i,j):
    if not i&j:return 1<<(i|j)
    h=1<<((i&j).bit_length()-1)
    low=nim_basis(i^h,j^h)
    return nim_reference(low,1<<h)^nim_reference(low,1<<(h-1))

def nim_reference(a,b):
    result=0
    while a:
        bit=a&-a;i=bit.bit_length()-1;a-=bit;remaining=b
        while remaining:
            second=remaining&-remaining;remaining-=second;result^=nim_basis(i,second.bit_length()-1)
    return result

def lyndon(word):return bool(word) and all(word<word[i:]+word[:i]for i in range(1,len(word)))
def factorization(s):
    if not s:return []
    answers=[]
    for mask in range(1<<(len(s)-1)):
        ends=[0]+[i for i in range(1,len(s))if mask>>(i-1)&1]+[len(s)];factors=[s[l:r]for l,r in zip(ends,ends[1:])]
        if all(lyndon(f)for f in factors) and all(a>=b for a,b in zip(factors,factors[1:])):answers.append(list(zip(ends,ends[1:])))
    assert len(answers)==1,(s,answers)
    return answers[0]

def common(a,b):
    result=[0,0,0]
    for j in range(len(b)):
        for i in range(len(a)):
            length=0
            while i+length<len(a) and j+length<len(b) and a[i+length]==b[j+length]:length+=1
            if length>result[2]:result=[i,j,length]
    return result

def period_reference(values,repeats,cap):
    n=len(values)
    for start in range(n):
        for p in range(1,min(cap,(n-start)//repeats)+1):
            if all(values[i]==values[start+(i-start)%p]for i in range(start,n)):return [start,p]
    return [-1]

def subtraction(moves,terms):
    values=[]
    for n in range(terms):
        seen={values[n-d]for d in moves if d<=n};g=0
        while g in seen:g+=1
        values.append(g)
    return values

def verify(run,binary):
    rng=random.Random(321325);commands=[];checks=[]
    enc=lambda s:s.hex()or '-'
    flat=lambda xs:' '.join(map(str,xs))
    def add(command,kind,want):commands.append(command);checks.append((kind,want))
    strings=[bytes(x)for n in range(9)for x in product(range(2),repeat=n)]
    strings += [bytes(rng.randrange(256)for _ in range(rng.randrange(35)))for _ in range(150)]
    patterns=[bytes(x)for n in range(5)for x in product(range(2),repeat=n)]
    for s in strings:
        queries=patterns+[s,s[:2],bytes([255])];want=[len({s[i:j]for i in range(len(s))for j in range(i+1,len(s)+1)}),len(s)]
        for p in queries:
            places=[i for i in range(len(s)+1)if s[i:i+len(p)]==p and i+len(p)<=len(s)];want.extend([int(bool(places)),len(places),places[0]if places else -1])
        add(f'S {enc(s)} {len(queries)}\n'+' '.join(map(enc,queries))+'\n','sam',want)
        if len(s)<=8:
            f=factorization(s);add(f'L {enc(s)}\n','exact',[len(f)]+[x for pair in f for x in pair])
    short=[bytes(x)for n in range(5)for x in product(range(2),repeat=n)]
    pairs=[(a,b)for a in short for b in short]+[(bytes(rng.randrange(7)for _ in range(rng.randrange(40))),bytes(rng.randrange(7)for _ in range(rng.randrange(40))))for _ in range(260)]
    for a,b in pairs:add(f'C {enc(a)} {enc(b)}\n','exact',common(a,b))
    for _ in range(450):
        h,w=rng.randrange(7),rng.randrange(7);w=w if h else 0;grid=[bytes(rng.randrange(256)for _ in range(w))for _ in range(h)];top,bottom=sorted([rng.randrange(h+1),rng.randrange(h+1)]);left,right=sorted([rng.randrange(w+1),rng.randrange(w+1)]);rb,cb=[rng.choice([257,911382323,972663749,1000000006])for _ in range(2)]
        want=[sum((grid[i][j]+1)*pow(rb,bottom-1-i,p)*pow(cb,right-1-j,p)for i in range(top,bottom)for j in range(left,right))%p for p in [1000000007,1000000009]]
        add(f'H {h} {rb} {cb}\n'+' '.join(map(enc,grid))+f'\n{top} {bottom} {left} {right}\n','exact',want)
    for i in range(64):
        for j in range(64):add(f'N {1<<i} {1<<j}\n','exact',[nim_basis(i,j)])
    for _ in range(260):
        a,b=rng.getrandbits(64),rng.getrandbits(64);add(f'N {a} {b}\n','exact',[nim_reference(a,b)])
    for _ in range(650):
        n=rng.randrange(70);a=[rng.randrange(5)for _ in range(n)]
        if _%2:
            start=rng.randrange(n+1);p=1+rng.randrange(9);pattern=[rng.randrange(5)for i in range(p)];a[start:]=[pattern[i%p]for i in range(n-start)]
        repeats=2+rng.randrange(4);cap=rng.randrange(30)
        add(f'P {n} {repeats} {cap}\n{flat(a)}\n','exact',period_reference(a,repeats,cap))
    for mask in range(256):
        moves=[i+1 for i in range(8)if mask>>i&1];terms=1+rng.randrange(100);values=subtraction(moves,terms);p=None
        if not moves:p=(0,1)
        else:
            for length in range(1,terms):
                same=0
                while same<terms-length and values[terms-1-same]==values[terms-1-length-same]:same+=1
                if same>=max(moves):
                    candidate=(terms-length-same,length)
                    if p is None or candidate<p:p=candidate
        index=rng.choice([rng.randrange(terms),rng.randrange(1000),MASK]);value=-1
        if index<terms:value=values[index]
        elif p:
            # 独立した有限状態の辞書で最初の同一窓を見つけ、未来の値を照合。
            if not moves:value=0
            else:
                extended=subtraction(moves,5000);windows={};m=max(moves)
                for end in range(m-1,len(extended)):
                    window=tuple(extended[end-m+1:end+1])
                    if window in windows:
                        old=windows[window];value=extended[old+(index-old)%(end-old)]if index>=old else extended[index];break
                    windows[window]=end
                assert value>=0
        add(f'G {len(moves)} {terms} {index}\n{flat(moves)}\n','exact',([p[0],p[1]]if p else [-1,0])+[value,terms]+values)
    for _ in range(160):
        n=1+rng.randrange(20);k=1+rng.randrange(8);seed=rng.getrandbits(64);raw=splitmix(seed);tokens=[[next(raw)for j in range(k)]for i in range(n)];state=[rng.randrange(k)for i in range(n)];initial=state[:];changes=[];want=[]
        for step in range(31):
            h=0
            for i,x in enumerate(state):h^=tokens[i][x]
            want.append(h)
            if step<30:pos=rng.randrange(n);v=rng.randrange(k);changes.extend([pos,v]);state[pos]=v
        add(f'Z {n} {k} {seed} 30\n{flat(initial)}\n{flat(changes)}\n','exact',want)
    for _ in range(500):
        n=1+rng.randrange(12);start=rng.randrange(n);goal=rng.randrange(2);parameter=rng.randrange(7);budget=rng.randrange(15);scores=[rng.choice([-(1<<63),(1<<63)-1,rng.randrange(-20,21)])for i in range(n)];keys=[rng.randrange(max(1,n//2))for i in range(n)];graph=[[rng.randrange(n)for j in range(rng.randrange(8))]for i in range(n)]
        better=lambda a,b:a>b if goal else a<b
        for op in 'BT':
            q=max(1,parameter)if op=='B'else parameter
            command=f'{op} {n} {start} {goal} {q} {budget}\n{flat(scores)}\n{flat(keys)}\n'+'\n'.join(str(len(row))+' '+flat(row)for row in graph)+'\n'
            if op=='B':
                frontier=[start];depth=0;evaluations=1;termination=1
                for depth_step in range(budget):
                    children=[child for parent in frontier for child in graph[parent]];evaluations+=len(children)
                    if not children:termination=3;break
                    ordered=sorted(enumerate(children),key=lambda pair:(-scores[pair[1]]if goal else scores[pair[1]],pair[0]));kept=[];seen=set()
                    for i,child in ordered:
                        if keys[child]not in seen:kept.append(child);seen.add(keys[child])
                        if len(kept)==q:break
                    frontier=kept;depth+=1
                want=[depth,evaluations,termination,len(frontier)]+[x for child in frontier for x in [child,scores[child]]]
            else:
                current=best=start;iterations=accepted=0;evaluations=1;termination=0;history={keys[start]:-1}
                for step in range(budget):
                    admissible=[]
                    for child in graph[current]:
                        evaluations+=1;forbidden=q>0 and keys[child]in history and step-history[keys[child]]<=q
                        if not forbidden or better(scores[child],scores[best]):admissible.append(child)
                    iterations+=1
                    if not admissible:termination=3;break
                    chosen=admissible[0]
                    for child in admissible[1:]:
                        if better(scores[child],scores[chosen]):chosen=child
                    current=chosen;accepted+=1;history[keys[current]]=step
                    if better(scores[current],scores[best]):best=current
                want=[best,current,scores[best],scores[current],iterations,evaluations,accepted,termination]
            add(command,'exact',want)
    for _ in range(200):
        initial=rng.randrange(-100,101);target=rng.randrange(-30,31);n=rng.randrange(40);moves=[(rng.randrange(-10,11),rng.randrange(2))for i in range(n)];state=initial;want=[]
        for d,accept in moves:
            if accept:state+=d
            want.extend([state,(state-target)**2])
        add(f'D {initial} {target} {n}\n'+'\n'.join(f'{d} {a}'for d,a in moves)+'\n','exact',want)
    actual=run([binary],input=''.join(commands)).splitlines();assert len(actual)==len(checks),(len(actual),len(checks))
    for line,(kind,want)in zip(actual,checks):
        tokens=list(map(int,line.split()))
        if kind=='sam':
            n=want[1];assert tokens[0]==want[0] and 1<=tokens[1]<=max(2,2*n-1) and tokens[2:]==want[2:],(tokens,want)
        else:assert tokens==want,(tokens,want)
    return len(checks)
