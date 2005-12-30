	.file	"test.cc"
	.text
	.align 2
.globl _Z11compute_msePA2_dS0_Pii
	.type	_Z11compute_msePA2_dS0_Pii, @function
_Z11compute_msePA2_dS0_Pii:
.LFB77:
	pushl	%ebp
.LCFI0:
	pxor	%xmm2, %xmm2
	movl	%esp, %ebp
.LCFI1:
	pushl	%ebx
.LCFI2:
	subl	$12, %esp
.LCFI3:
	movl	20(%ebp), %eax
	movl	8(%ebp), %ebx
	movl	12(%ebp), %ecx
	testl	%eax, %eax
	jle	.L8
	xorl	%edx, %edx
.L6:
	movsd	(%ebx,%edx), %xmm3
	movsd	8(%ebx,%edx), %xmm0
	subsd	(%ecx,%edx), %xmm3
	subsd	8(%ecx,%edx), %xmm0
	addl	$16, %edx
	mulsd	%xmm3, %xmm3
	mulsd	%xmm0, %xmm0
	subl	$1, %eax
	addsd	%xmm0, %xmm3
	addsd	%xmm3, %xmm2
	jne	.L6
.L8:
	movsd	%xmm2, -16(%ebp)
	fldl	-16(%ebp)
	addl	$12, %esp
	popl	%ebx
	popl	%ebp
	ret
.LFE77:
	.size	_Z11compute_msePA2_dS0_Pii, .-_Z11compute_msePA2_dS0_Pii
	.section	.rodata.str1.1,"aMS",@progbits,1
.LC4:
	.string	"mse = %g\n"
.LC5:
	.string	"%d %d\n"
	.text
	.align 2
.globl main
	.type	main, @function
main:
.LFB78:
	pushl	%ebp
.LCFI4:
	movl	%esp, %ebp
.LCFI5:
	pushl	%edi
.LCFI6:
	movl	$1, %edi
	pushl	%esi
.LCFI7:
	pushl	%ebx
.LCFI8:
	subl	$28, %esp
.LCFI9:
	andl	$-16, %esp
	movl	$5776, (%esp)
	call	malloc
	movl	$5776, (%esp)
	movl	%eax, %esi
	call	malloc
	movl	$1444, (%esp)
	movl	%eax, %ebx
	call	malloc
	xorl	%ecx, %ecx
	movl	%eax, -16(%ebp)
	movl	$0, -20(%ebp)
.L16:
	movl	-20(%ebp), %edx
	movl	-16(%ebp), %eax
	cvtsi2sd	%edi, %xmm1
	movl	%edx, -4(%eax,%edi,4)
	cvtsi2sd	%edx, %xmm3
	movsd	%xmm1, (%ebx,%ecx)
	movl	$361, %eax
	movsd	%xmm3, (%esi,%ecx)
	subl	%edx, %eax
	movl	%edi, %edx
	cvtsi2sd	%eax, %xmm2
	movl	$360, %eax
	subl	-20(%ebp), %eax
	movsd	%xmm2, 8(%esi,%ecx)
	movl	%edi, -20(%ebp)
	cvtsi2sd	%eax, %xmm1
	addl	$1, %edi
	movsd	%xmm1, 8(%ebx,%ecx)
	addl	$16, %ecx
	cmpl	$360, %edx
	jle	.L16
	pxor	%xmm2, %xmm2
	xorl	%eax, %eax
	movl	$360, %edx
.L21:
	movsd	(%esi,%eax), %xmm4
	movsd	8(%esi,%eax), %xmm0
	subsd	(%ebx,%eax), %xmm4
	subsd	8(%ebx,%eax), %xmm0
	addl	$16, %eax
	mulsd	%xmm4, %xmm4
	mulsd	%xmm0, %xmm0
	subl	$1, %edx
	addsd	%xmm0, %xmm4
	addsd	%xmm4, %xmm2
	jns	.L21
	movsd	%xmm2, 4(%esp)
	movl	$.LC4, (%esp)
	call	printf
	movl	%ebx, 8(%esp)
	movl	%esi, 4(%esp)
	movl	$.LC5, (%esp)
	call	printf
	movl	%esi, (%esp)
	call	free
	movl	%ebx, (%esp)
	call	free
	movl	-16(%ebp), %eax
	movl	%eax, (%esp)
	call	free
	leal	-12(%ebp), %esp
	popl	%ebx
	xorl	%eax, %eax
	popl	%esi
	popl	%edi
	popl	%ebp
	ret
.LFE78:
	.size	main, .-main
	.ident	"GCC: (GNU) 3.3 20030226 (prerelease) (SuSE Linux)"
