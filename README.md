# Dynamic-CARP
A Novel Generalised Meta-Heuristic Framework for Dynamic Capacitated Arc Routing Problems

This repository contains the code for the paper *A Novel Generalised Meta-Heuristic Framework for Dynamic Capacitated Arc Routing Problems*. This paper is now under review, but its original version has been on arxiv. 


**Reference**
```
@misc{tong2021novel,
      title={A Novel Generalised Meta-Heuristic Framework for Dynamic Capacitated Arc Routing Problems}, 
      author={Hao Tong and Leandro L. Minku and Stefan Menzel and Bernhard Sendhoff and Xin Yao},
      year={2021},
      eprint={2104.06585},
      archivePrefix={arXiv},
      primaryClass={cs.NE}
}
```

**Source Code**

- This repository contains 4 different algorithms in total. Two algorithm ***MAENS***$^{[1]}$ and ***TSA***$^{[2]}$(***RTS****$^{[3]}$) comes from the [Dr. Yi Mei](https://github.com/meiyi1986) and code for these two algorithm is downloaded from [this](https://meiyi1986.github.io/publication/).
- Algorithms ***ILMA***$^{[4]}$ and ***MASDC***$^{[5]}$ are implemented by ourselves according to their original paper.
- ***Virtual Task*** strategy and ***sequence transfer strategy*** are implemented in `function.c`. ***Service simulator*** is implemented in `simulaor.c`
- The main function for three experiments discussed in the paper are implemented in `main.c`


**Usage**

We have provided the `CMakeLists.txt` in the source code. You can clone or download this repository and use `gcc` + `cmake` to compile an executable program.

**References**

[1]. K. Tang, Y. Mei, and X. Yao, “Memetic algorithm with extended neighborhood search for capacitated arc routing problems,” IEEE Transactions
on Evolutionary Computation, vol. 13, no. 5, pp. 1151–1166, 2009.

[2]. J. Brandao and R. Eglese, “A deterministic tabu search algorithm for the ˜
capacitated arc routing problem,” Computers & Operations Research,
vol. 35, no. 4, pp. 1112–1126, 2008.

[3]. Y. Mei, K. Tang, and X. Yao, “A global repair operator for capacitated
arc routing problem,” IEEE Transactions on Systems, Man, and Cybernetics, Part B (Cybernetics), vol. 39, no. 3, pp. 723–734, 2009.

[4]. Y. Mei, K. Tang, and X. Yao, “Improved memetic algorithm for capacitated arc routing problem,” in 2009 IEEE Congress on Evolutionary Computation. IEEE,
2009, pp. 1699–1706.

[5]. M. Liu, H. K. Singh, and T. Ray, “A memetic algorithm with a new split
scheme for solving dynamic capacitated arc routing problems,” in 2014
IEEE Congress on Evolutionary Computation (CEC). IEEE, 2014, pp.
595–602.

