#!/bin/bash

file_list=`find ./kernel/`
echo $file_list > file_list.txt

for file in $file_list
do
	#echo "$file"
	if [ -d $file ]; then
	    echo "skip $file"
	else
	    #sed -i -e '/BU5D[0-9]\{5\}/d' $file
	    #sed -i -e '/BK4D[0-9]\{5\}/d' $file
	    #sed -i -e '/DTS[0-9]\{13\}/d' $file
	    #sed -i -e '/h00105634/d' $file
	    #modify for update qualcom 2110 baseline begin
	    #modify for wifi baseline 20091126 begin
	    #sed -i -e 's/lxy: //' $file
	    
	    
	    #sed -i -e '/fengqiu/d' $file	
	    #sed -i -e '/hanshirong/d' $file
	    
	    #sed -i -e '/wanghao/d' $file 
	    #sed -i -e '/yanghaimin/d' $file	    
	    #sed -i -e 'dingzhipeng' $file
	    #sed -i -e '/wangping/d' $file	 
	    #sed -i -e '/xuwei/d' $file 
	    #sed -i -e '/liwei/d' $file
	    #sed -i -e '/libeibei/d' $file	
	    #sed -i -e '/likang/d' $file
	    #sed -i -e '/guojing/d' $file	
	    #sed -i -e '/yangxiaocong/d' $file   
	    #sed -i -e '/liaihua/d' $file
	    #sed -i -e '/sunwenyong/d' $file      	        	      
	    #sed -i -e '/yuezenglong/d' $file 
	    #sed -i -e '/lijiasheng/d' $file
	    #sed -i -e '/jiangweizheng/d' $file	    
	    #sed -i -e '/sunchenggang/d' $file  
	    #sed -i -e '/cuixuefeng/d' $file
	    #sed -i -e '/lijianzhao/d' $file	    
	    #sed -i -e '/zhaoyuxia/d' $file 
	    #sed -i -e '/chenjikun/d' $file 
	    #sed -i -e '/yuhaitao/d' $file
	    #sed -i -e '/zhuwenying/d' $file	    
	    #sed -i -e '/zhangpeng/d' $file
	    #sed -i -e '/mengdong/d' $file	    
	    #sed -i -e '/duhongyan/d' $file 
	    #sed -i -e '/taohanwen/d' $file
	fi
done
