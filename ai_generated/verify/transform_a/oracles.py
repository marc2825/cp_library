"""Independent definition, permutation determinant, enumerated spans and solutions."""
from itertools import permutations, product
import random

def det(a):
    n=len(a);answer=0
    for perm in permutations(range(n)):
        term=(-1)**sum(perm[i]>perm[j] for i in range(n) for j in range(i+1,n))
        for i,j in enumerate(perm):term*=a[i][j]
        answer+=term
    return answer

def charpoly(a,p):
    n=len(a);answer=[0]*(n+1)
    for perm in permutations(range(n)):
        poly=[(-1)**sum(perm[i]>perm[j] for i in range(n) for j in range(i+1,n))]
        for i,j in enumerate(perm):
            q=[0]*(len(poly)+1)
            for k,x in enumerate(poly):
                q[k]-=x*a[i][j]
                if i==j:q[k+1]+=x
            poly=q
        answer=[(x+y)%p for x,y in zip(answer,poly)]
    return answer

def span(rows,d,p):
    result={tuple([0]*d)}
    for row in rows:result={tuple((x+c*y)%p for x,y in zip(v,row)) for v in result for c in range(p)}
    return result

def xor_span(rows):
    result={0}
    for row in rows:result|={x^row for x in result}
    return result

def verify(run,binary):
    rng=random.Random(316320);commands=[];checks=[]
    def add(command,check):commands.append(command);checks.append(check)
    flat=lambda a:' '.join(map(str,a))
    matrix=lambda a:' '.join(flat(row) for row in a)
    vector=lambda xs:[len(xs)]+xs
    for _ in range(200):
        n,m=rng.randrange(180),rng.randrange(180);a=[rng.randrange(-50,51) for _ in range(n)];b=[rng.randrange(-50,51) for _ in range(m)];c=[0]*(n+m-1) if n*m else []
        for i,x in enumerate(a):
            for j,y in enumerate(b):c[i+j]+=x*y
        add(f'K {n} {m}\n{flat(a)}\n{flat(b)}\n',('exact',vector(c)))
    for _ in range(120):
        n=1<<rng.randrange(8);a=[rng.randrange(-30,31) for _ in range(n)];b=[rng.randrange(-30,31) for _ in range(n)]
        w=[sum((-1)**((i&j).bit_count())*x for j,x in enumerate(a)) for i in range(n)]
        add(f'W {n} 0\n{flat(a)}\n',('exact',vector(w)));add(f'W {n} 1\n{flat(w)}\n',('exact',vector(a)))
        for op in range(3):
            c=[0]*n
            for i,x in enumerate(a):
                for j,y in enumerate(b):c[(i&j) if op==0 else (i|j) if op==1 else (i^j)]+=x*y
            add(f'B {n} {op}\n{flat(a)}\n{flat(b)}\n',('exact',vector(c)))
        ranked=[sum(x for j,x in enumerate(a) if j&s==j and j.bit_count()==k) for k in range(n.bit_length()) for s in range(n)]
        add(f'R {n}\n{flat(a)}\n',('exact',vector(ranked)))
    for p in [2,3,5,998244353]:
        matrices=[[]]
        for n in range(1,6):matrices.extend([[[rng.randrange(-p,p) for _ in range(n)] for _ in range(n)] for _ in range(35)])
        if p==2:matrices.extend([[list(row[:2]),list(row[2:])] for row in product(range(2),repeat=4)])
        for a in matrices:
            n=len(a);add(f'I {p} {n}\n{matrix(a)}\n',('inverse',a,p));add(f'C {p} {n}\n{matrix(a)}\n',('exact',vector(charpoly(a,p))))
    for p in [2,3]:
        for _ in range(200):
            d=rng.randrange(6);r,s=rng.randrange(4),rng.randrange(4);a=[[rng.randrange(p) for _ in range(d)]for _ in range(r)];b=[[rng.randrange(p) for _ in range(d)]for _ in range(s)]
            add(f'S {p} {d} {r} {s}\n{matrix(a)}\n{matrix(b)}\n',('space',span(a,d,p)&span(b,d,p),d,p))
    systems=[(3,[[(mask>>(i*3+j))&1 for j in range(3)] for i in range(3)],[(rhs>>i)&1 for i in range(3)]) for mask in range(512) for rhs in range(8)]
    for _ in range(180):
        m=rng.randrange(8);n=rng.randrange(7);systems.append((m,[[rng.randrange(2) for _ in range(m)]for _ in range(n)],[rng.randrange(2)for _ in range(n)]))
    for m,a,b in systems:
        rows=[sum(x<<j for j,x in enumerate(row))for row in a];solutions={x for x in range(1<<m) if all((x&row).bit_count()%2==rhs for row,rhs in zip(rows,b))}
        add(f'G {len(a)} {m}\n'+ '\n'.join(''.join(map(str,row)) for row in a)+'\n'+flat(b)+'\n',('gf2',solutions,len(xor_span(rows)).bit_length()-1,m))
    for _ in range(300):
        a=[rng.getrandbits(64 if _%2 else 5)for i in range(rng.randrange(9))];b=[rng.getrandbits(64 if _%2 else 5)for i in range(rng.randrange(9))]
        if _%3==0:b+=a[:2]
        add(f'X {len(a)} {len(b)}\n{flat(a)}\n{flat(b)}\n',('xor',xor_span(a)&xor_span(b)))
    lines=run([binary],input=''.join(commands)).splitlines();assert len(lines)==len(checks),(len(lines),len(checks))
    for line,check in zip(lines,checks):
        tokens=list(map(int,line.split())) if check[0]!='gf2' else line.split()
        if check[0]=='exact':assert tokens==check[1],(tokens,check)
        elif check[0]=='inverse':
            _,a,p=check;n=len(a)
            if det(a)%p==0:assert tokens==[-1],(tokens,check)
            else:
                assert tokens[0]==n*n and len(tokens)==1+n*n
                inverse=[tokens[1+i*n:1+(i+1)*n] for i in range(n)]
                assert all(0<=x<p for row in inverse for x in row)
                for u,v in [(a,inverse),(inverse,a)]:assert all(sum(u[i][k]*v[k][j] for k in range(n))%p==(i==j) for i in range(n)for j in range(n)),(tokens,check)
        elif check[0]=='space':
            _,want,d,p=check;r,c=tokens[:2];assert c==d and len(tokens)==2+r*d;basis=[tokens[2+i*d:2+(i+1)*d]for i in range(r)];actual=span(basis,d,p);assert actual==want and p**r==len(actual),(tokens,check)
        elif check[0]=='gf2':
            _,want,rank,m=check;assert int(tokens[0])==rank and bool(int(tokens[1]))==bool(want)
            if want:
                particular=int(tokens[2],2);count=int(tokens[3]);basis=[int(x,2)for x in tokens[4:]];assert count==m-rank==len(basis);actual=xor_span(basis);assert len(actual)==2**count and {particular^x for x in actual}==want
        else:
            _,want=check;count=tokens[0];basis=tokens[1:];assert len(basis)==count and xor_span(basis)==want and 2**count==len(want)
    return len(checks)
