// Version directive is set by Warzone when loading the shader
// (This shader supports GLSL 1.20 - 1.50 core.)

uniform vec4 color;
uniform vec4 structurePosition;

#if (!defined(GL_ES) && (__VERSION__ >= 130)) || (defined(GL_ES) && (__VERSION__ >= 300))
out vec4 FragColor;
#else
// Uses gl_FragColor
#endif

void main()
{
	#if (!defined(GL_ES) && (__VERSION__ >= 130)) || (defined(GL_ES) && (__VERSION__ >= 300))
	FragColor = vec4(color.xyz, 1 - distance(gl_FragCoord.xyz, structurePosition.xyz));
	#else
	gl_FragColor = vec4(color.xyz, 1 - distance(gl_FragCoord.xyz, structurePosition.xyz));
	#endif
}
