%THIS FUNCTION GIVES VARIOUS CASES AS INPUT
function cases()

%CASE1:DEL_R_ORDERED=10DEGREE
%CASE2:DEL_R_ORDERED=15DEGREE
%CASE3:ZIG-ZAG MOTION: DEL_R_ORDERED=10DEGREE,WHEN PSI=10DEGREE
%        DEL_R_ORDERED=-10DEGREE
%CASE4:SAME AS CASE 3 WITH DEL_R_ORDERED=-10DEGREE INTIALLY
%CASE5:AKA CASE3:DEL_R_ORDERED=20DEGREE
%CASE6:AKA CASE4_2:DEL_R_ORDERED=-20DEGREE
%CASE7:zig zag pitch motion de
%CASE8:??



n=8;%TOTAL NO OF CASES

for i=1:n
fname=num2str(i,'%0d') ;
euler(fname);
end
end