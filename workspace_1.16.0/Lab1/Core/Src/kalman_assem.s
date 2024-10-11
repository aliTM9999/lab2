/*
* kalman_assem.s
*/
//.section .data

.syntax unified //as.pdf : p141
.align  16 //as.pdf :p71
.section .text, "x" //as.pdf :p96
//.rodata
.global kalman_assem //as.pdf : p254
/**
Comments for code readability
*/
//Your assembly code
kalman_assem: //label

//PUSH current state to stack
//setup registers
//your function
//POP stack

push {r7}
mov r2,#0 //counter to move through array
add r3, r0, r2, lsl #2 //getting q and putting address in r3
vldr s1, [r3] //putting value of q in s1

add r2, r2, #1
add r4, r0, r2, lsl #2 //getting r and putting address in r4
vldr s2, [r4] //putting value of r in s2

add r2, r2, #1
add r5, r0, r2, lsl #2 //getting p and putting address in r5
vldr s3, [r5] //putting value of p in s3

add r2, r2, #1
add r6, r0, r2, lsl #2 //getting x and putting address in r6
vldr s4, [r6] //putting value of x in s4

add r2, r2, #1
add r7, r0, r2, lsl #2 //getting k and putting address in r7
vldr s5, [r7] //putting value of k in s5

//p <- p + q
vadd.f32 s3,s3,s1 // s3 contains updated p

//k <- p / (p+r)
vadd.f32 s6,s3,s2
vdiv.f32 s5,s3,s6 // s5 contains updated k
vstr s5, [r7]

//x <- x + k*(measurement - x)
vsub.f32 s6,s0,s4
vmul.f32 s6,s5,s6
vadd.f32 s4,s4,s6 // s4 contains updated x
vstr s4, [r6]
vmov s0, s4 // to return the value of x

//p <- (1-k)*p
vmov s6, #1.0
vsub.f32 s6,s6,s5
vmul.f32 s3,s3,s6 // s3 contains updated p
vstr s3, [r5]

//checking if overflow, underflow or division by zero
vmrs r0, fpscr
mov r1, #0xE  // 0xE corresponds to bits 1, 2, and 3
and r2, r0, r1
mov r3, #0
cmp r2, r3
bne iferror

pop {r7}
bx lr

iferror:
mov r0, #0
sub r0, r0, #1//move -1 in r0 because of either overflow, underflow or division by 0
b iferror //infinite loop to stop program




//extern void kalman_assem(float input, float* kstate);


//q r p x k
