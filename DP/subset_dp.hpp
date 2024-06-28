/*
★ Name         : 部分集合の部分集合のO(3^n)での走査
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description : 各部分集合iの中の全ての部分集合jをO(3^n)で走査するテクニック
 

 ■ Usage       :


 ■ Verify      :
 ■ References  :

 ■ TODO        :
*/


#pragma once


using namespace std;


for (int i = 0; i < (1 << N); i++) {
    for (int j = i; j >= 0; j--) {
        j &= i;

    }
}