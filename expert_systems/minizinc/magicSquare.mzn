include "alldifferent.mzn";
int:N;
var int:Rowsum;
var int:Columnsum;
var int:DiagonlSum;
var int:OppDiagonalSum;

set of int: MagicSquareRange = 1..N;
set of int: ValueRange = 1..(N*N);
array[MagicSquareRange, MagicSquareRange] of var ValueRange: MagicSquare;

constraint alldifferent([MagicSquare[i,j] | i, j in MagicSquareRange]);

constraint forall(i in MagicSquareRange) (sum(j in MagicSquareRange) (MagicSquare[i,j]) = Rowsum) 
           /\ forall(i in MagicSquareRange)(sum(j in MagicSquareRange) (MagicSquare[j,i]) = Columnsum); 

constraint sum(i in MagicSquareRange) (MagicSquare[i,i]) = DiagonlSum 
          /\ sum(i in MagicSquareRange) (MagicSquare[i,N-i+1]) = OppDiagonalSum;

constraint Rowsum = Columnsum /\ DiagonlSum = OppDiagonalSum /\ Rowsum = DiagonlSum;
 
solve satisfy;

output["ColumnSum:" ++show(Columnsum)++"  RowSum:" ++show(Rowsum)++ "  DiagonlSum:"++ show(DiagonlSum)++ "  OppDiagonalSum:" ++ show(OppDiagonalSum)++"\n"];

output[show(MagicSquare)]