

__constant sampler_t smpUNormNoClampNearest = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;
__constant sampler_t smpNormClampLinear = CLK_NORMALIZED_COORDS_TRUE | CLK_ADDRESS_CLAMP | CLK_FILTER_LINEAR;

__constant float REF_SAMPLING_INTERVAL = 150.f;

__kernel void raycaster(read_only image3d_t volume
                        , read_only image2d_t entryPoints
                        , read_only image2d_t exitPoints
                        , read_only image2d_t transferFunction
                        , float stepSize
                        , uint3 volumeDimension
                        , write_only image2d_t output) 
{
    int2 globalId = (int2)(get_global_id(0), get_global_id(1)); 
    //uint3 volumeDimension = (uint3)(128,128, 128);
    //if( any(globalId >= get_image_dim(output)) ) {
    //    return;
    //}
    float4 entry = read_imagef(entryPoints, smpUNormNoClampNearest, globalId); 
    
    float4 result = 0.f;
    if(entry.w != 0) {
        float4 exit = read_imagef(exitPoints, smpUNormNoClampNearest, globalId); 
        float3 direction = exit.xyz - entry.xyz;
        float tEnd = length(direction);
        float tIncr = 1.f/(stepSize*length(direction*convert_float3(volumeDimension)));
        direction = normalize(direction);
        float3 p = entry.xyz;
        float t = 0;
        float volumeSample;
        while(t <= tEnd) {
            float3 pos = entry.xyz+t*direction;
            volumeSample = read_imagef(volume, smpNormClampLinear, as_float4(pos)).w; 
            float4 color = read_imagef(transferFunction, smpNormClampLinear, (float2)(volumeSample, 0.5f));
            color.w = 1.f - pow(1.f - color.w, tIncr * REF_SAMPLING_INTERVAL);
			result.xyz = result.xyz + (1.0 - result.w) * color.w * color.xyz;
			result.w = result.w + (1.0 -result.w) * color.w;	
            //result += color;
            t += tIncr;

        }
        // Riemann sum
        //result *= tIncr; 
        // Remove last part due to steppping too far. 
        //result -= (t-tEnd)*volumeSample;
        //result.w = 1.f;
        //result.xy = exit.xy;
        //result.x = 1.f;
    }
    
    //float4 result = read_imagef(entryPoints, smpUNormNoClampNearest, globalId) + convert_float(globalId.x)/255.f; 
    //float4 result = convert_float4(read_imageui(entryPoints, smpUNormNoClampNearest, globalId)) + convert_float(globalId.x)/255.f;
    //float4 result = (float4)(0.5f, 0.5f, 0.f, 1.f);
    write_imagef(output, globalId,  result);    
    //write_imageh(output, globalId,  (half4)(0.5f, 0.5f, 0.f, 1.f));    
    //write_imageui(output, globalId,  convert_uint4(255.f*result));    
}

