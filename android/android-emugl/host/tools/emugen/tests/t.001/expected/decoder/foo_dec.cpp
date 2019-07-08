// Generated Code - DO NOT EDIT !!
// generated by 'emugen'


#include <string.h>
#include "foo_opcodes.h"

#include "foo_dec.h"


#include "ProtocolUtils.h"

#include "ChecksumCalculatorThreadInfo.h"

#include <stdio.h>

typedef unsigned int tsize_t; // Target "size_t", which is 32-bit for now. It may or may not be the same as host's size_t when emugen is compiled.

#  define DEBUG(...) do { if (emugl_cxt_logger) { emugl_cxt_logger(__VA_ARGS__); } } while(0)
#ifdef CHECK_GL_ERRORS
#  define SET_LASTCALL(name)  sprintf(lastCall, #name)
#else
#  define SET_LASTCALL(name)
#endif
using namespace emugl;

size_t foo_decoder_context_t::decode(void *buf, size_t len, IOStream *stream, ChecksumCalculator* checksumCalc) {
	if (len < 8) return 0; 
#ifdef CHECK_GL_ERRORS
	char lastCall[256] = {0};
#endif
	unsigned char *ptr = (unsigned char *)buf;
	const unsigned char* const end = (const unsigned char*)buf + len;
    const size_t checksumSize = checksumCalc->checksumByteSize();
    const bool useChecksum = checksumSize > 0;
	while (end - ptr >= 8) {
		uint32_t opcode = *(uint32_t *)ptr;   
		int32_t packetLen = *(int32_t *)(ptr + 4);
		if (end - ptr < packetLen) return ptr - (unsigned char*)buf;
		switch(opcode) {
		case OP_fooAlphaFunc: {
			FooInt var_func = Unpack<FooInt,uint32_t>(ptr + 8);
			FooFloat var_ref = Unpack<FooFloat,uint32_t>(ptr + 8 + 4);
			if (useChecksum) {
				ChecksumCalculatorThreadInfo::validOrDie(checksumCalc, ptr, 8 + 4 + 4, ptr + 8 + 4 + 4, checksumSize, 
					"foo_decoder_context_t::decode, OP_fooAlphaFunc: GL checksumCalculator failure\n");
			}
			DEBUG("foo(%p): fooAlphaFunc(%d %f )\n", stream, var_func, var_ref);
			this->fooAlphaFunc(var_func, var_ref);
			SET_LASTCALL("fooAlphaFunc");
			break;
		}
		case OP_fooIsBuffer: {
			uint32_t size_stuff __attribute__((unused)) = Unpack<uint32_t,uint32_t>(ptr + 8);
			InputBuffer inptr_stuff(ptr + 8 + 4, size_stuff);
			if (useChecksum) {
				ChecksumCalculatorThreadInfo::validOrDie(checksumCalc, ptr, 8 + 4 + size_stuff, ptr + 8 + 4 + size_stuff, checksumSize, 
					"foo_decoder_context_t::decode, OP_fooIsBuffer: GL checksumCalculator failure\n");
			}
			size_t totalTmpSize = sizeof(FooBoolean);
			totalTmpSize += checksumSize;
			unsigned char *tmpBuf = stream->alloc(totalTmpSize);
			mOnBeginReadback();
			DEBUG("foo(%p): fooIsBuffer(%p(%u) )\n", stream, (void*)(inptr_stuff.get()), size_stuff);
			*(FooBoolean *)(&tmpBuf[0]) = 			this->fooIsBuffer((void*)(inptr_stuff.get()));
			if (useChecksum) {
				ChecksumCalculatorThreadInfo::writeChecksum(checksumCalc, &tmpBuf[0], totalTmpSize - checksumSize, &tmpBuf[totalTmpSize - checksumSize], checksumSize);
			}
			stream->flush();
			mOnEndReadback();
			SET_LASTCALL("fooIsBuffer");
			break;
		}
		case OP_fooUnsupported: {
			uint32_t size_params __attribute__((unused)) = Unpack<uint32_t,uint32_t>(ptr + 8);
			InputBuffer inptr_params(ptr + 8 + 4, size_params);
			if (useChecksum) {
				ChecksumCalculatorThreadInfo::validOrDie(checksumCalc, ptr, 8 + 4 + size_params, ptr + 8 + 4 + size_params, checksumSize, 
					"foo_decoder_context_t::decode, OP_fooUnsupported: GL checksumCalculator failure\n");
			}
			DEBUG("foo(%p): fooUnsupported(%p(%u) )\n", stream, (void*)(inptr_params.get()), size_params);
			this->fooUnsupported((void*)(inptr_params.get()));
			SET_LASTCALL("fooUnsupported");
			break;
		}
		case OP_fooDoEncoderFlush: {
			FooInt var_param = Unpack<FooInt,uint32_t>(ptr + 8);
			if (useChecksum) {
				ChecksumCalculatorThreadInfo::validOrDie(checksumCalc, ptr, 8 + 4, ptr + 8 + 4, checksumSize, 
					"foo_decoder_context_t::decode, OP_fooDoEncoderFlush: GL checksumCalculator failure\n");
			}
			DEBUG("foo(%p): fooDoEncoderFlush(%d )\n", stream, var_param);
			this->fooDoEncoderFlush(var_param);
			SET_LASTCALL("fooDoEncoderFlush");
			break;
		}
		case OP_fooTakeConstVoidPtrConstPtr: {
			uint32_t size_param __attribute__((unused)) = Unpack<uint32_t,uint32_t>(ptr + 8);
			InputBuffer inptr_param(ptr + 8 + 4, size_param);
			if (useChecksum) {
				ChecksumCalculatorThreadInfo::validOrDie(checksumCalc, ptr, 8 + 4 + size_param, ptr + 8 + 4 + size_param, checksumSize, 
					"foo_decoder_context_t::decode, OP_fooTakeConstVoidPtrConstPtr: GL checksumCalculator failure\n");
			}
			DEBUG("foo(%p): fooTakeConstVoidPtrConstPtr(%p(%u) )\n", stream, (const void* const*)(inptr_param.get()), size_param);
			this->fooTakeConstVoidPtrConstPtr((const void* const*)(inptr_param.get()));
			SET_LASTCALL("fooTakeConstVoidPtrConstPtr");
			break;
		}
		case OP_fooSetComplexStruct: {
			uint32_t size_obj __attribute__((unused)) = Unpack<uint32_t,uint32_t>(ptr + 8);
			InputBuffer inptr_obj(ptr + 8 + 4, size_obj);
			void* inptr_obj_unpacked;
			 FooStruct unpacked; inptr_obj_unpacked = (void*)(&unpacked); fooStructUnpack((unsigned char*)(inptr_obj.get()), size_obj, inptr_obj_unpacked);
			if (useChecksum) {
				ChecksumCalculatorThreadInfo::validOrDie(checksumCalc, ptr, 8 + 4 + size_obj, ptr + 8 + 4 + size_obj, checksumSize, 
					"foo_decoder_context_t::decode, OP_fooSetComplexStruct: GL checksumCalculator failure\n");
			}
			DEBUG("foo(%p): fooSetComplexStruct(%p(%u) )\n", stream, (const FooStruct*)(inptr_obj.get()), size_obj);
			this->fooSetComplexStruct((const FooStruct*)(inptr_obj_unpacked));
			SET_LASTCALL("fooSetComplexStruct");
			break;
		}
		case OP_fooGetComplexStruct: {
			uint32_t size_obj __attribute__((unused)) = Unpack<uint32_t,uint32_t>(ptr + 8);
			if (useChecksum) {
				ChecksumCalculatorThreadInfo::validOrDie(checksumCalc, ptr, 8 + 4, ptr + 8 + 4, checksumSize, 
					"foo_decoder_context_t::decode, OP_fooGetComplexStruct: GL checksumCalculator failure\n");
			}
			size_t totalTmpSize = size_obj;
			totalTmpSize += checksumSize;
			unsigned char *tmpBuf = stream->alloc(totalTmpSize);
			mOnBeginReadback();
			OutputBuffer outptr_obj(&tmpBuf[0], size_obj);
			void* forPacking_obj = nullptr;
			 FooStruct tmp; forPacking_obj = (void*)tmp;
			DEBUG("foo(%p): fooGetComplexStruct(%p(%u) )\n", stream, (FooStruct*)(outptr_obj.get()), size_obj);
			this->fooGetComplexStruct((FooStruct*)(forPacking_obj));
			if (size_obj) {
			 fooStructPack((unsigned char*)outptr_obj.get(), (FooStruct*)forPacking_obj); }
			outptr_obj.flush();
			if (useChecksum) {
				ChecksumCalculatorThreadInfo::writeChecksum(checksumCalc, &tmpBuf[0], totalTmpSize - checksumSize, &tmpBuf[totalTmpSize - checksumSize], checksumSize);
			}
			stream->flush();
			mOnEndReadback();
			SET_LASTCALL("fooGetComplexStruct");
			break;
		}
		case OP_fooInout: {
			uint32_t size_count __attribute__((unused)) = Unpack<uint32_t,uint32_t>(ptr + 8);
			InputBuffer inptr_count(ptr + 8 + 4, size_count);
			if (useChecksum) {
				ChecksumCalculatorThreadInfo::validOrDie(checksumCalc, ptr, 8 + 4 + size_count, ptr + 8 + 4 + size_count, checksumSize, 
					"foo_decoder_context_t::decode, OP_fooInout: GL checksumCalculator failure\n");
			}
			size_t totalTmpSize = size_count;
			totalTmpSize += checksumSize;
			unsigned char *tmpBuf = stream->alloc(totalTmpSize);
			mOnBeginReadback();
			OutputBuffer outptr_count(&tmpBuf[0], size_count);
			memcpy(outptr_count.get(), inptr_count.get(), size_count);
			DEBUG("foo(%p): fooInout(%p(%u) )\n", stream, (uint32_t*)(outptr_count.get()), size_count);
			this->fooInout((uint32_t*)(outptr_count.get()));
			outptr_count.flush();
			if (useChecksum) {
				ChecksumCalculatorThreadInfo::writeChecksum(checksumCalc, &tmpBuf[0], totalTmpSize - checksumSize, &tmpBuf[totalTmpSize - checksumSize], checksumSize);
			}
			stream->flush();
			mOnEndReadback();
			SET_LASTCALL("fooInout");
			break;
		}
		default:
			return ptr - (unsigned char*)buf;
		} //switch
		ptr += packetLen;
	} // while
	return ptr - (unsigned char*)buf;
}
