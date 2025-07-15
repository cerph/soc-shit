rm -f a.out
iverilog -s core_tb -o a.out -c cmdfile -g relative-include -DDEFINE_ME
vvp a.out
