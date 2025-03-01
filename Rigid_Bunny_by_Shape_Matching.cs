using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Rigid_Bunny_by_Shape_Matching : MonoBehaviour
{	
	int length;
	public bool launched = false;
	Vector3[] X; // universal vertex positions
	Vector3[] Q; // relative vertex positions
	Vector3[] Y; // temporary vertex positions
	Vector3[] V; // velocities
	Vector3[] F; // positions of the particles
	Matrix4x4 QQt = Matrix4x4.zero;

	public float linear_decay	= 0.999f;				// for velocity decay
	public float restitution 	= 0.5f;					// for collision
	public float friction = 0.2f;
	public Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
	


    // Start is called before the first frame update
    void Start()
    {
    	Mesh mesh = GetComponent<MeshFilter>().mesh;
        V = new Vector3[mesh.vertices.Length];
        X = mesh.vertices;
        Q = mesh.vertices;
		Y = mesh.vertices;
		F = mesh.vertices;
		length = X.Length;

        //Centerizing Q.
        Vector3 c=Vector3.zero;
        for(int i=0; i<Q.Length; i++)
        	c+=Q[i];
        c/=Q.Length;
        for(int i=0; i<Q.Length; i++)
        	Q[i]-=c;

        //Get QQ^t ready.
		for(int i=0; i<Q.Length; i++)
		{
			QQt[0, 0]+=Q[i][0]*Q[i][0];
			QQt[0, 1]+=Q[i][0]*Q[i][1];
			QQt[0, 2]+=Q[i][0]*Q[i][2];
			QQt[1, 0]+=Q[i][1]*Q[i][0];
			QQt[1, 1]+=Q[i][1]*Q[i][1];
			QQt[1, 2]+=Q[i][1]*Q[i][2];
			QQt[2, 0]+=Q[i][2]*Q[i][0];
			QQt[2, 1]+=Q[i][2]*Q[i][1];
			QQt[2, 2]+=Q[i][2]*Q[i][2];
		}
		QQt[3, 3]=1;
 
		for(int i=0; i<X.Length; i++)
			V[i][0]=4.0f;

		Update_Mesh(transform.position, Matrix4x4.Rotate(transform.rotation), 0);
		transform.position=Vector3.zero;
		transform.rotation=Quaternion.identity;
   	}

   	// Polar Decomposition that returns the rotation from F.
   	Matrix4x4 Get_Rotation(Matrix4x4 F)
	{
		Matrix4x4 C = Matrix4x4.zero;
	    for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        C[ii,jj]+=F[kk,ii]*F[kk,jj];
	   
	   	Matrix4x4 C2 = Matrix4x4.zero;
		for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        C2[ii,jj]+=C[ii,kk]*C[jj,kk];
	    
	    float det    =  F[0,0]*F[1,1]*F[2,2]+
	                    F[0,1]*F[1,2]*F[2,0]+
	                    F[1,0]*F[2,1]*F[0,2]-
	                    F[0,2]*F[1,1]*F[2,0]-
	                    F[0,1]*F[1,0]*F[2,2]-
	                    F[0,0]*F[1,2]*F[2,1];
	    
	    float I_c    =   C[0,0]+C[1,1]+C[2,2];
	    float I_c2   =   I_c*I_c;
	    float II_c   =   0.5f*(I_c2-C2[0,0]-C2[1,1]-C2[2,2]);
	    float III_c  =   det*det;
	    float k      =   I_c2-3*II_c;
	    
	    Matrix4x4 inv_U = Matrix4x4.zero;
	    if(k<1e-10f)
	    {
	        float inv_lambda=1/Mathf.Sqrt(I_c/3);
	        inv_U[0,0]=inv_lambda;
	        inv_U[1,1]=inv_lambda;
	        inv_U[2,2]=inv_lambda;
	    }
	    else
	    {
	        float l = I_c*(I_c*I_c-4.5f*II_c)+13.5f*III_c;
	        float k_root = Mathf.Sqrt(k);
	        float value=l/(k*k_root);
	        if(value<-1.0f) value=-1.0f;
	        if(value> 1.0f) value= 1.0f;
	        float phi = Mathf.Acos(value);
	        float lambda2=(I_c+2*k_root*Mathf.Cos(phi/3))/3.0f;
	        float lambda=Mathf.Sqrt(lambda2);
	        
	        float III_u = Mathf.Sqrt(III_c);
	        if(det<0)   III_u=-III_u;
	        float I_u = lambda + Mathf.Sqrt(-lambda2 + I_c + 2*III_u/lambda);
	        float II_u=(I_u*I_u-I_c)*0.5f;
	        
	        
	        float inv_rate, factor;
	        inv_rate=1/(I_u*II_u-III_u);
	        factor=I_u*III_u*inv_rate;
	        
	       	Matrix4x4 U = Matrix4x4.zero;
			U[0,0]=factor;
	        U[1,1]=factor;
	        U[2,2]=factor;
	        
	        factor=(I_u*I_u-II_u)*inv_rate;
	        for(int i=0; i<3; i++)
	        for(int j=0; j<3; j++)
	            U[i,j]+=factor*C[i,j]-inv_rate*C2[i,j];
	        
	        inv_rate=1/III_u;
	        factor=II_u*inv_rate;
	        inv_U[0,0]=factor;
	        inv_U[1,1]=factor;
	        inv_U[2,2]=factor;
	        
	        factor=-I_u*inv_rate;
	        for(int i=0; i<3; i++)
	        for(int j=0; j<3; j++)
	            inv_U[i,j]+=factor*U[i,j]+inv_rate*C[i,j];
	    }
	    
	    Matrix4x4 R=Matrix4x4.zero;
	    for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        R[ii,jj]+=F[ii,kk]*inv_U[kk,jj];
	    R[3,3]=1;
	    return R;
	}

	Matrix4x4 Vector3_Multiplication(Vector3 a, Vector3 b)
	{
		Matrix4x4 A = Matrix4x4.zero;
		for(int i = 0; i < 3; i++)
			for(int j = 0; j < 3; j++)
				A[i, j] = a[i] * b[j];
		A[3, 3] = 1.0f;
		return A;
	}

	
	

	Matrix4x4 Matrix4x4_Addition(Matrix4x4 A, Matrix4x4 B)
	{
		Matrix4x4 C = Matrix4x4.zero;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				C[i, j] = A[i, j] + B[i, j]; 
		return C;
	}

	// Update the mesh vertices according to translation c and rotation R.
	// It also updates the velocity.
	void Update_Mesh(Vector3 c, Matrix4x4 R, float inv_dt)
   	{
   		for(int i = 0; i < Q.Length; i++)
		{
			Vector3 x=(Vector3)(R * Q[i]) + c;
			V[i] = (x - X[i]) * inv_dt;
			X[i] = x;			
		}	
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices = X ;
   	}

	// Perform simple particle collision. P is the collision plane, N is the normal of the plane.
	void Collision(Vector3 P, Vector3 N, float inv_dt)
	{
		for (int i = 0; i < length; i++)
		{	
			float d = Vector3.Dot(Y[i] - P, N);
			if (d < 0.0f) // if the particle is inside the collision plane
			{
				// Update the position of the particle
				Y[i] -= d * N;
				Vector3 vi_velocity = V[i];
				if (Vector3.Dot(vi_velocity, N) < 0.0f) // if the particle is moving into the collision plane
				{
					Vector3 vi_velocityN = Vector3.Dot(vi_velocity, N) * N;
					Vector3 vi_velocityT = vi_velocity - vi_velocityN;
					float a = Math.Max(0.0f, 1.0f - friction * (1.0f + restitution) * vi_velocityN.magnitude / vi_velocityT.magnitude);

					// Update the velocity of the particle
					Vector3 vi_velocityN_new = -1.0f * restitution * vi_velocityN;
					Vector3 vi_velocityT_new = a * vi_velocityT;
					V[i] = vi_velocityN_new + vi_velocityT_new;
					
				}
				
			}
		}
	}

    // Update is called once per frame
    void Update()
    {	
		//Step 0: Game Control
        if (Input.GetKey("r"))
        {
            // return initial state
            launched = false;
			transform.position = new Vector3(0, 0.6f, 0);
            for (int i = 0; i < length; i++)
            {

                V[i] = new Vector3(0.0f, 0.0f, 0.0f);
            }
			
            // Update_Mesh(new Vector3(0, 0.6f, 0),
            //             Matrix4x4.Rotate(Quaternion.Euler(new Vector3(80, 0, 0))), 0);

			
        }
        if (Input.GetKey("l"))
        {
            launched = true;
            for (int i = 0; i < length; i++)
            {
                V[i] = new Vector3(10.0f, 4.0f, 0.0f);
            }
        }

        if (!launched)
            return;

		float dt = 0.03f;
		float inv_dt = 1.0f / dt;

  		//Step 1: run a simple particle system.
        for(int i = 0; i < length; i++)
        {
			V[i] += dt * gravity;
			V[i] *= linear_decay;
			Y[i] = X[i] + dt * V[i];
        }

        //Step 2: Perform simple particle collision.
		Collision(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0), inv_dt);
		Collision(new Vector3(2.0f, 0, 0), new Vector3(-1, 0, 0), inv_dt);

		// Step 3: Use shape matching to get new translation c and 
		// new rotation R. Update the mesh by c and R.
        //Shape Matching (translation)
		Vector3 c = Vector3.zero; 
		for(int i = 0; i < length; i++)
		{			
			c += Y[i];
		}
		c /= length;
		//Shape Matching (rotation)
		Matrix4x4 A = Matrix4x4.zero;
		for(int i = 0; i < length; i++)
		{
			A = Matrix4x4_Addition(A, Vector3_Multiplication( Y[i] - c, Q[i]));
		}
		A[3,3] = 1.0f;
		Matrix4x4 R = Get_Rotation(A * QQt.inverse);
		Update_Mesh(c, R, inv_dt);

		//Update the normals of the triangles.
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.RecalculateNormals();


    }
}
