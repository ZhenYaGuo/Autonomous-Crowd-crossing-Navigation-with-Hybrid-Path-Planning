function result=test_recursive(n)

if n==0
    result=0;
else
    result=test_recursive(n-1)+1;
end
